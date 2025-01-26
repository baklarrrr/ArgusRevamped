import sys
import psutil
import platform
import math
import time
import dearpygui.dearpygui as dpg

"""
Updated version that replaces 'add_drawing' with 'add_drawlist'.
We also remove 'draw_circle', 'draw_line', etc. calls that might be missing in older DPG versions.
We replicate them with polylines.

If you see WMI COM errors, ensure OpenHardwareMonitor is installed and running with WMI,
otherwise it uses simulated data.
"""

# ==================== MONITORING LOGIC =====================

class SystemMonitor:
    def __init__(self):
        self.update_interval = 2  # seconds
        self.temperature_history = {}
        self.fan_speed_history = {}
        self.cpu_load_history = []
        self.max_history_length = 60
        self.sensor_warning_shown = False
        self.sensors = self._init_sensors()

    def _init_sensors(self):
        if platform.system() == "Windows":
            return self._init_windows_sensors()
        else:
            return self._init_unix_sensors()

    def _init_windows_sensors(self):
        try:
            import wmi
            w = wmi.WMI(namespace="root\\OpenHardwareMonitor")
            return w.Sensor()
        except ImportError:
            if not self.sensor_warning_shown:
                print("Install 'wmi' and OpenHardwareMonitor for sensor data on Windows")
                self.sensor_warning_shown = True
            return []
        except Exception as e:
            print(f"Sensor init error: {str(e)}")
            return []

    def _init_unix_sensors(self):
        sensors = []
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = float(f.read()) / 1000
                sensors.append(("Linux_CPU", "Temperature", temp))
        except:
            pass
        return sensors

    def get_sensor_data(self):
        data = {
            'cpu_temp': 0.0,
            'gpu_temp': 0.0,
            'system_temp': 0.0,
            'fan_speeds': {},
            'cpu_load': psutil.cpu_percent(),
            'ram_usage': psutil.virtual_memory().percent
        }

        if platform.system() == "Windows":
            for sensor in self.sensors:
                try:
                    if sensor.SensorType == 'Temperature':
                        value = float(sensor.Value)
                        if 'CPU' in sensor.Name:
                            data['cpu_temp'] = max(data['cpu_temp'], value)
                        elif 'GPU' in sensor.Name:
                            data['gpu_temp'] = max(data['gpu_temp'], value)
                        else:
                            data['system_temp'] = max(data['system_temp'], value)
                    elif sensor.SensorType == 'Fan':
                        data['fan_speeds'][sensor.Name] = float(sensor.Value)
                except:
                    continue
        else:
            for name, type_, value in self.sensors:
                if type_ == 'Temperature':
                    data['cpu_temp'] = max(data['cpu_temp'], value)

        if data['cpu_temp'] == 0:
            data['cpu_temp'] = 30 + psutil.cpu_percent() * 0.5
        if data['gpu_temp'] == 0:
            data['gpu_temp'] = 40 + (psutil.cpu_percent() * 0.3)
        if not data['fan_speeds']:
            data['fan_speeds'] = {'Simulated Fan': 1500 + psutil.cpu_percent() * 20}

        self._update_history('cpu_temp', data['cpu_temp'])
        self._update_history('gpu_temp', data['gpu_temp'])
        self._update_history('system_temp', data['system_temp'])
        for fan, speed in data['fan_speeds'].items():
            self._update_history(fan, speed)

        self.cpu_load_history.append(data['cpu_load'])
        self.cpu_load_history = self.cpu_load_history[-self.max_history_length:]

        return data

    def _update_history(self, sensor, value):
        if sensor not in self.temperature_history:
            self.temperature_history[sensor] = []
        self.temperature_history[sensor].append(value)
        self.temperature_history[sensor] = self.temperature_history[sensor][-self.max_history_length:]

        if 'Fan' in sensor:
            if sensor not in self.fan_speed_history:
                self.fan_speed_history[sensor] = []
            self.fan_speed_history[sensor].append(value)
            self.fan_speed_history[sensor] = self.fan_speed_history[sensor][-self.max_history_length:]


class FanController:
    def __init__(self):
        self.fan_profiles = {}
        self.active_profile = None
        self.virtual_mode = True  # default
        self.virtual_fans = {
            'CPU Fan': 1200,
            'Case Fan 1': 800,
            'GPU Fan': 1500
        }

    def set_fan_speed(self, fan_name, rpm):
        if self.virtual_mode:
            self.virtual_fans[fan_name] = rpm
            return True
        try:
            if platform.system() == "Windows":
                import wmi
                w = wmi.WMI(namespace="root\\WMI")
                fans = w.FanSpeed()
                for f in fans:
                    if f.Name == fan_name:
                        f.DesiredSpeed = rpm
                        return True
            return False
        except Exception as e:
            print(f"Fan control error: {str(e)}")
            return False

    def create_profile(self, name, curve):
        self.fan_profiles[name] = curve

    def auto_adjust_fans(self, sensor_data):
        if not self.active_profile:
            return
        for fan, curve in self.active_profile.items():
            temp = sensor_data.get(curve['source'], 0)
            rpm = self._calculate_rpm(temp, curve['points'])
            self.set_fan_speed(fan, rpm)

    def _calculate_rpm(self, temp, points):
        sorted_points = sorted(points.items())
        for i in range(len(sorted_points) - 1):
            x1, y1 = sorted_points[i]
            x2, y2 = sorted_points[i + 1]
            if x1 <= temp <= x2:
                return y1 + (temp - x1) * (y2 - y1) / (x2 - x1)
        if temp < sorted_points[0][0]:
            return sorted_points[0][1]
        else:
            return sorted_points[-1][1]


class PIDController:
    def __init__(self, Kp=0.8, Ki=0.2, Kd=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.reset()

    def reset(self):
        self.last_error = 0
        self.integral = 0

    def compute(self, setpoint, process_variable, dt=1):
        error = setpoint - process_variable
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output


# ==================== DEAR PYGGUI APP =====================

monitor = SystemMonitor()
fan_controller = FanController()
pid = PIDController()

last_update_time = 0.0

# We'll replicate the gauges using drawlists.
# We'll also replicate shapes with polylines for older DPG.

def draw_circle_polyline(parent, cx, cy, radius, color=(255,255,255,255), fill=None, thickness=1, segments=64):
    """ A rough circle via polyline. """
    pts = []
    import math
    for i in range(segments + 1):
        angle = 2 * math.pi * i / segments
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        pts.append((x, y))
    if fill:
        # fill requires closed polygon; let's do a draw_polygon
        dpg.draw_polygon(pts, color=color, fill=fill, thickness=thickness, parent=parent)
    else:
        dpg.draw_polyline(pts, color=color, thickness=thickness, closed=True, parent=parent)


def draw_line_polyline(parent, p1, p2, color=(255,255,255,255), thickness=1):
    dpg.draw_polyline([p1, p2], color=color, thickness=thickness, closed=False, parent=parent)


def draw_text_simple(parent, pos, text, color=(255,255,255,255)):
    # If dpg.draw_text is missing, we can skip or find some alternative.
    dpg.draw_text(pos=pos, text=text, color=color, parent=parent)


def draw_arc_polyline(parent, cx, cy, radius, start_angle_deg, end_angle_deg, color=(255,255,255,255), thickness=2, segments=30):
    import math
    points = []
    start_rad = math.radians(-start_angle_deg)
    end_rad = math.radians(-end_angle_deg)
    step = (end_rad - start_rad) / segments if segments else 0
    for i in range(segments + 1):
        angle = start_rad + i * step
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        points.append((x, y))
    dpg.draw_polyline(points, color=color, thickness=thickness, closed=False, parent=parent)


def draw_gauge(parent, value, min_val, max_val, title):
    dpg.delete_item(parent, children_only=True)

    fraction = 0.0
    if max_val > min_val:
        fraction = (value - min_val) / (max_val - min_val)
        fraction = max(0, min(1, fraction))

    start_angle = 150
    span_angle = 240
    current_angle = start_angle + fraction * span_angle

    cx, cy = 100, 100
    r = 70

    # background circle
    draw_circle_polyline(
        parent=parent,
        cx=cx,
        cy=cy,
        radius=r,
        color=(60,60,60,255),
        fill=(40,40,40,255),
        thickness=2,
        segments=50
    )

    # arc
    if current_angle > start_angle:
        draw_arc_polyline(
            parent=parent,
            cx=cx,
            cy=cy,
            radius=r - 5,
            start_angle_deg=start_angle,
            end_angle_deg=current_angle,
            color=(0,180,150,255),
            thickness=6,
            segments=30
        )

    # needle
    import math
    ang_rad = math.radians(-current_angle)
    needle_len = r * 0.6
    x_end = cx + needle_len * math.cos(ang_rad)
    y_end = cy + needle_len * math.sin(ang_rad)

    draw_line_polyline(
        parent=parent,
        p1=(cx, cy),
        p2=(x_end, y_end),
        color=(255,50,50,255),
        thickness=3
    )

    # text
    draw_text_simple(
        parent=parent,
        pos=(cx - 40, cy + 50),
        text=f"{title}: {value:.1f}",
        color=(255,255,255,255)
    )


def build_ui():
    with dpg.window(tag="MainWindow", label="Hardware Monitor", width=1200, height=800):
        with dpg.tab_bar():
            with dpg.tab(label="Dashboard"):
                dpg.add_text("Gauges:")
                with dpg.group(horizontal=True):
                    # replaced add_drawing with add_drawlist
                    dpg.add_drawlist(width=200, height=200, tag="cpu_gauge")
                    dpg.add_drawlist(width=200, height=200, tag="gpu_gauge")
                    dpg.add_drawlist(width=200, height=200, tag="fan_gauge")

                dpg.add_spacer(height=20)
                dpg.add_text("Temperature History")
                with dpg.plot(label="Temp History", height=300, width=500, tag="temp_plot"):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="Time", tag="temp_x_axis")
                    with dpg.plot_axis(dpg.mvYAxis, label="Temperature (C)", tag="temp_y_axis"):
                        dpg.add_line_series([], [], label="CPU Temp", tag="temp_series")

                dpg.add_spacer(height=20)
                dpg.add_text("Fan Speed History")
                with dpg.plot(label="Fan History", height=300, width=500, tag="fan_plot"):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="Time", tag="fan_x_axis")
                    with dpg.plot_axis(dpg.mvYAxis, label="Speed (RPM)", tag="fan_y_axis"):
                        dpg.add_line_series([], [], label="Fan Speed", tag="fan_series")

            with dpg.tab(label="Fan Control"):
                dpg.add_text("(Demo) No advanced curve editing yet, just a placeholder.")
                dpg.add_checkbox(label="Automatic Fan Control", tag="auto_mode_check")
                dpg.add_button(label="Apply Fan Profile", callback=lambda: print("Applying fan profile..."))

            with dpg.tab(label="Settings"):
                dpg.add_slider_int(label="Update Interval (s)", default_value=2, min_value=1, max_value=10, tag="update_interval")
                dpg.add_checkbox(label="Virtual Mode (No HW Access)", default_value=True, tag="virtual_mode_check")


def update_data():
    global last_update_time
    current_time = time.time()
    interval = dpg.get_value("update_interval")

    if current_time - last_update_time < interval:
        return

    last_update_time = current_time
    data = monitor.get_sensor_data()

    fan_controller.virtual_mode = dpg.get_value("virtual_mode_check")

    if dpg.get_value("auto_mode_check"):
        fan_controller.auto_adjust_fans(data)

    cpu_val = data['cpu_temp']
    gpu_val = data['gpu_temp']
    if data['fan_speeds']:
        some_fan_speed = list(data['fan_speeds'].values())[0]
    else:
        some_fan_speed = 0.0

    # call draw_gauge directly each frame
    draw_gauge("cpu_gauge", cpu_val, 0, 100, "CPU Temp")
    draw_gauge("gpu_gauge", gpu_val, 0, 100, "GPU Temp")
    draw_gauge("fan_gauge", some_fan_speed, 0, 3000, "Fan RPM")

    # CPU temp history
    cpu_hist = monitor.temperature_history.get('cpu_temp', [])
    x_vals = list(range(len(cpu_hist)))
    dpg.set_value("temp_series", [x_vals, cpu_hist])
    if cpu_hist:
        dpg.set_axis_limits("temp_y_axis", min(cpu_hist), max(cpu_hist) + 5)
        dpg.set_axis_limits("temp_x_axis", 0, len(cpu_hist))

    # fan speed history
    if monitor.fan_speed_history:
        fan_name, speeds = next(iter(monitor.fan_speed_history.items()))
        fx_vals = list(range(len(speeds)))
        dpg.set_value("fan_series", [fx_vals, speeds])
        if speeds:
            dpg.set_axis_limits("fan_y_axis", min(speeds), max(speeds) + 100)
            dpg.set_axis_limits("fan_x_axis", 0, len(speeds))


def main():
    dpg.create_context()
    build_ui()
    dpg.create_viewport(title="Hardware Monitor App (Dear PyGui)", width=1280, height=800)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    while dpg.is_dearpygui_running():
        update_data()
        dpg.render_dearpygui_frame()

    dpg.destroy_context()


if __name__ == "__main__":
    main()
