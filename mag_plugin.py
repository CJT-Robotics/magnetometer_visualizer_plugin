import rospy
import numpy as np

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QFormLayout, QDoubleSpinBox, QSpinBox, QComboBox, QTabWidget, QGroupBox, QPushButton)
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QIcon
import os
import rospkg

from sensor_msgs.msg import MagneticField

class SensorDataModel:
    def __init__(self, max_points=500):
        self.max_points = max_points
        self.data = {
            'x': np.zeros(self.max_points),
            'y': np.zeros(self.max_points),
            'z': np.zeros(self.max_points)
        }
        self.latest_vals = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
    def resize(self, new_size):
        for key in self.data:
            if new_size > self.max_points:
                pad_width = new_size - self.max_points
                self.data[key] = np.pad(self.data[key], (pad_width, 0), 'constant')
            else:
                self.data[key] = self.data[key][-new_size:]
        self.max_points = new_size

    def add_data(self, x, y, z):
        self.latest_vals = {'x': x, 'y': y, 'z': z}
        for key, val in zip(['x', 'y', 'z'], [x, y, z]):
            self.data[key] = np.roll(self.data[key], -1)
            self.data[key][-1] = val

    def get_max_absolute(self):
        return max(abs(v) for v in self.latest_vals.values())


class MagPlotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(tight_layout=True)
        super(MagPlotCanvas, self).__init__(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True, linestyle=':', alpha=0.6)
        
        self.lines = {
            'x': self.ax.plot([], [], 'r-', label='X-Axis', linewidth=1.2)[0],
            'y': self.ax.plot([], [], 'g-', label='Y-Axis', linewidth=1.2)[0],
            'z': self.ax.plot([], [], 'b-', label='Z-Axis', linewidth=1.2)[0]
        }

        self.status_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes, verticalalignment='top', fontsize=11, fontweight='bold', bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))
        
        self.ax.legend(loc='upper right', fontsize='small')
        self.fill_area = None

    def update_plot(self, model, threshold, zoom, factor, unit, status_msg, is_alarm):
        self.ax.set_ylabel(f"Magnetic Field [{unit}]")
        self.ax.set_xlim(0, model.max_points)
        self.ax.set_ylim(-zoom, zoom)

        x_axis = range(model.max_points)
        for key in ['x', 'y', 'z']:
            self.lines[key].set_data(x_axis, model.data[key] * factor)
            
        self.status_text.set_text(status_msg)
        if is_alarm:
            self.status_text.get_bbox_patch().set_facecolor('#ffcccc')
            self.status_text.set_color('darkred')
        else:
            self.status_text.get_bbox_patch().set_facecolor('#e6f3ff')
            self.status_text.set_color('black')

        if self.fill_area:
            self.fill_area.remove()
            self.fill_area = None
            
        mask = (np.abs(model.data['x'] * factor) > threshold) | \
               (np.abs(model.data['y'] * factor) > threshold) | \
               (np.abs(model.data['z'] * factor) > threshold)
        if np.any(mask):
            self.fill_area = self.ax.fill_between(x_axis, -zoom, zoom, where=mask, color='yellow', alpha=0.2)
        self.draw_idle()

class MagPlugin(Plugin):
    def __init__(self, context):
        super(MagPlugin, self).__init__(context)
        self.setObjectName('MagPlugin')

        self.model = SensorDataModel(max_points=500)
        self.sub = None
        
        self._setup_ui(context)
        
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._sync_gui)
        self.update_timer.start(40)

    def init_plugin(self, context):
        if self._main_widget:
            title = "Magnetic Field Monitor - CJT Robotics"
            if context.serial_number() > 1:
                title += f" ({context.serial_number()})"
            
            self._main_widget.setWindowTitle(title)
            self._main_widget.setObjectName(title)

    def _setup_ui(self, context):
        self._main_widget = QWidget()
        main_layout = QVBoxLayout(self._main_widget)
        
        self.tabs = QTabWidget()

        try:
            rp = rospkg.RosPack()
            package_path = rp.get_path('magnetometer_visualizer_plugin') 
            icon_path = os.path.join(package_path, 'res', 'favicon.png')
            self._main_widget.setWindowIcon(QIcon(icon_path))
        except Exception as e:
            rospy.logwarn(e)

        self._main_widget.setWindowTitle('Magnetometer Visualizer')
        
        self.plot_tab = QWidget()
        plot_layout = QVBoxLayout(self.plot_tab)
        self.canvas = MagPlotCanvas()
        plot_layout.addWidget(self.canvas)

        self.settings_tab = QWidget()
        settings_layout = QVBoxLayout(self.settings_tab)
        config_group = QGroupBox("Sensor & Display Configuration")
        form_layout = QFormLayout(config_group)
        
        self.topic_combo = QComboBox()
        self.topic_combo.setEditable(True)
        self.topic_combo.currentIndexChanged.connect(self._reconnect_topic)
        
        self.refresh_btn = QPushButton("Refresh Topics")
        self.refresh_btn.clicked.connect(self._update_topic_list)
        
        self.unit_combo = QComboBox()
        self.unit_combo.addItems(["Tesla (T)", "Millitesla (mT)", "Mikrotesla (uT)", "Nanotesla (nT)"])
        self.unit_combo.setCurrentIndex(0)
        
        self.points_spin = QSpinBox()
        self.points_spin.setRange(50, 5000); self.points_spin.setValue(250)
        self.points_spin.valueChanged.connect(lambda v: self.model.resize(v))
        
        self.thresh_spin = QDoubleSpinBox()
        self.thresh_spin.setRange(0.0, 10000.0); self.thresh_spin.setValue(5.0) 
        
        self.zoom_spin = QDoubleSpinBox()
        self.zoom_spin.setRange(0.1, 10000.0); self.zoom_spin.setValue(150.0)
        
        form_layout.addRow("Topic:", self.topic_combo)
        form_layout.addRow("", self.refresh_btn)
        form_layout.addRow("Unit Scaling:", self.unit_combo)
        form_layout.addRow("Buffer:", self.points_spin)
        form_layout.addRow("Threshold:", self.thresh_spin)
        form_layout.addRow("Y-Axis:", self.zoom_spin)
        
        settings_layout.addWidget(config_group)
        settings_layout.addStretch()

        self.tabs.addTab(self.plot_tab, "Live Graph")
        self.tabs.addTab(self.settings_tab, "Settings")
        
        main_layout.addWidget(self.tabs)
        context.add_widget(self._main_widget)
        self._update_topic_list()
        self._reconnect_topic()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic', self.topic_combo.currentText())
        instance_settings.set_value('buffer', self.points_spin.value())
        instance_settings.set_value('threshold', self.thresh_spin.value())
        instance_settings.set_value('zoom', self.zoom_spin.value())
        instance_settings.set_value('unit', self.unit_combo.currentIndex())

    def restore_settings(self, plugin_settings, instance_settings):
        topic = instance_settings.value('topic', '')
        self.topic_combo.setEditText(topic)
        self.points_spin.setValue(int(instance_settings.value('buffer', 250)))
        self.thresh_spin.setValue(float(instance_settings.value('threshold', 5.0)))
        self.zoom_spin.setValue(float(instance_settings.value('zoom', 150.0)))
        self.unit_combo.setCurrentIndex(int(instance_settings.value('unit', 0)))
        self._reconnect_topic()

    def _reconnect_topic(self):
        topic_name = self.topic_combo.currentText().strip()
        if self.sub: 
            self.sub.unregister()
        if topic_name:
            self.sub = rospy.Subscriber(topic_name, MagneticField, self._ros_callback)

    def _update_topic_list(self):
        current_text = self.topic_combo.currentText()
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        
        try:
            all_topics = rospy.get_published_topics()
            mag_topics = [t[0] for t in all_topics if t[1] == 'sensor_msgs/MagneticField']
            
            self.topic_combo.addItems(sorted(mag_topics))
            
            index = self.topic_combo.findText(current_text)
            if index >= 0:
                self.topic_combo.setCurrentIndex(index)
                
        except Exception as e:
            rospy.logerr(f"Could not update topics: {e}")
        
        self.topic_combo.blockSignals(False)

    def _ros_callback(self, msg):
        self.model.add_data(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)

    def _sync_gui(self):
        if not self._main_widget.isVisible(): return
            
        factors = [1.0, 1e3, 1e6, 1e9]
        units = ["T", "mT", "uT", "nT"]
        idx = self.unit_combo.currentIndex()
        
        f, u = factors[idx], units[idx]
        thr = self.thresh_spin.value()
        mx = self.model.get_max_absolute() * f
        alarm = mx > thr
        
        status = f"{'DETECTION' if alarm else 'MONITORING'}\nPeak: {mx:.2f} {u}"
        self.canvas.update_plot(self.model, thr, self.zoom_spin.value(), f, u, status, alarm)

    def shutdown_plugin(self):
        self.update_timer.stop()
        if self.sub: self.sub.unregister()