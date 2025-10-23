from machine import Pin, I2C, PWM, time_pulse_us
import network, socket, urequests, time, math
from lcd_api import LcdApi
from machine_i2c_lcd import I2cLcd

# ---------- USER CONFIG ----------
WIFI_SSID     = "Robotic WIFI"
WIFI_PASSWORD = "rbtWIFI@2025"
BOT_TOKEN     = "8389035011:AAFlmATmWS1WKwvg6GAA4sPv0AO5y1GDRjw"  # <<< REPLACE EXACTLY (no .replace, no edits)
ALLOWED_CHAT_IDS = {1375325558}

API = "https://api.telegram.org/bot" + BOT_TOKEN
SEND_MSG_URL = API + "/sendMessage"

# ---------- WIFI CONNECT ----------
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        while not wlan.isconnected():
            time.sleep(0.5)
            print(".", end="")
    print("\nConnected to WiFi:", WIFI_SSID)
    print("IP:", wlan.ifconfig()[0])
    return wlan.ifconfig()[0]

# ---------- TELEGRAM ----------
def _urlencode(d):
    parts = []
    for k, v in d.items():
        s = str(v)
        s = s.replace("%", "%25").replace(" ", "%20").replace("\n", "%0A")
        s = s.replace("&", "%26").replace("?", "%3F").replace("=", "%3D")
        parts.append(f"{k}={s}")
    return "&".join(parts)

def _telegram_send(text):
    # Silent: no console spam on Unauthorized or network errors
    for chat_id in ALLOWED_CHAT_IDS:
        try:
            payload = {"chat_id": chat_id, "text": text}
            url = SEND_MSG_URL + "?" + _urlencode(payload)
            r = urequests.get(url)
            # Only print when ok==True to avoid error noise
            try:
                if '"ok":true' in r.text:
                    print("[TELEGRAM] Sent OK")
            finally:
                r.close()
        except Exception:
            pass  # ignore errors silently

def send_telegram_receipt(ticket):
    ti = time.localtime(ticket.time_in)
    to = time.localtime(ticket.time_out)
    def fmt(t): return f"{t[3]:02d}:{t[4]:02d}:{t[5]:02d}"

    duration_sec = int(ticket.time_out - ticket.time_in)
    duration_min = round(duration_sec / 60, 2)
    fee_text = "Free (under 1 min)" if duration_sec < 60 else f"${ticket.fee}"

    message = (
        f"Parking Receipt\n"
        f"Slot: {ticket.slot}\n"
        f"ID: {ticket.id}\n"
        f"Time In: {fmt(ti)}\n"
        f"Time Out: {fmt(to)}\n"
        f"Duration: {duration_sec} sec ({duration_min} min)\n"
        f"Fee: {fee_text}"
    )
    _telegram_send(message)

# <<< TELEGRAM GATE ALERT >>>
def send_gate_alert(is_open):
    if is_open:
        _telegram_send("Gate is OPEN — Object detected ✅")
    else:
        _telegram_send("Gate is CLOSED — Area clear ⛔")

# ---------- LCD ----------
I2C_ADDR = 0x27
I2C_SCL = 22
I2C_SDA = 21
LCD_ROWS = 2
LCD_COLS = 16

i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, LCD_ROWS, LCD_COLS)

_last_lcd_text = ("", "")

def _occ_char(b):  # True=occupied / False=free
    return "O" if b else "F"

def lcd_show_compact():
    global _last_lcd_text
    s1 = _occ_char(slots[0].occupied)
    s2 = _occ_char(slots[1].occupied)
    s3 = _occ_char(slots[2].occupied)
    line1 = f"S1:{s1} S2:{s2} S3:{s3}"[:16]
    line2 = ("Gate:OPEN" if gate_is_open() else "Gate:CLOSED")[:16]
    if _last_lcd_text != (line1, line2):
        lcd.clear()
        lcd.move_to(0, 0); lcd.putstr(line1)
        lcd.move_to(0, 1); lcd.putstr(line2)
        _last_lcd_text = (line1, line2)

def lcd_show_temp(msg, delay=2):
    lcd.clear()
    lcd.putstr(msg)
    time.sleep(delay)
    lcd_show_compact()

# ---------- SERVO ----------
# <<< GATE SYSTEM >>> Smooth servo on D14
SERVO_PIN = 14
servo = PWM(Pin(SERVO_PIN), freq=50)

def _angle_to_duty16(angle):
    pulse_ms = 0.5 + (2.0 * angle / 180.0)  # 0.5..2.5ms over 20ms
    return int((pulse_ms / 20.0) * 65535)

def smooth_servo_move(target_angle, step_delay=0.006):
    try:
        current_duty = servo.duty_u16()
        current_angle = int(round((current_duty * 20.0 / 65535.0 - 0.5) * (180.0 / 2.0)))
        current_angle = min(180, max(0, current_angle))
    except:
        current_angle = 0
    step = 1 if target_angle > current_angle else -1
    for ang in range(current_angle, target_angle + step, step):
        servo.duty_u16(_angle_to_duty16(ang))
        time.sleep(step_delay)

_gate_open_state = False
def gate_is_open():
    return _gate_open_state

def gate_open():
    global _gate_open_state
    if not _gate_open_state:
        smooth_servo_move(90)
        _gate_open_state = True
        send_gate_alert(True)   # alert on transition

def gate_close():
    global _gate_open_state
    if _gate_open_state:
        smooth_servo_move(0)
        _gate_open_state = False
        send_gate_alert(False)  # alert on transition

# ---------- ULTRASONIC (Mode 1) ----------
US_TRIG = 27
US_ECHO = 26
US_CAR_DISTANCE_CM = 10
trig = Pin(US_TRIG, Pin.OUT, value=0)
echo = Pin(US_ECHO, Pin.IN)

def read_distance_cm(timeout_us=40000):
    trig.off(); time.sleep_us(2)
    trig.on(); time.sleep_us(10)
    trig.off()
    try:
        t = time_pulse_us(echo, 1, timeout_us)
        if t < 0:
            return None
        return (t / 2.0) / 29.1
    except OSError:
        return None

def car_at_gate():
    d = read_distance_cm()
    return (d is not None) and (d <= US_CAR_DISTANCE_CM)

# ---------- PARKING LOGIC ----------
# IR pins: S1=D19, S2=D23, S3=D34
IR_PINS = [19, 23, 34]
IR_ACTIVE_LEVEL = 0
DEBOUNCE_MS = 120
EXIT_GRACE_MS = 1000
PRICE_PER_MIN = 0.5
LCD_REFRESH_MS = 400

class Ticket:
    def __init__(self, ticket_id, slot_name, time_in):
        self.id = ticket_id
        self.slot = slot_name
        self.time_in = time_in
        self.time_out = None
        self.fee = 0.0
    def close(self, t_out):
        self.time_out = t_out
        duration_sec = int(self.time_out - self.time_in)
        if duration_sec < 60:
            self.fee = 0.0
        else:
            minutes_started = math.ceil(duration_sec / 60)
            self.fee = round(minutes_started * PRICE_PER_MIN, 2)
        return duration_sec, self.fee

class Slot:
    def __init__(self, pin_no, name):
        self.pin = Pin(pin_no, Pin.IN)
        self.name = name
        self._last_raw = self._read_raw()
        self._stable = self._last_raw
        self._last_change = time.ticks_ms()
        self.occupied = False
        self.assigned_id = None
        self.time_in = None
        self.exit_watch_started = None
        self.last_fee = 0.0
        self.last_duration = 0

    def _read_raw(self):
        v = self.pin.value()
        return 1 if v == IR_ACTIVE_LEVEL else 0

    def update(self, now_ms):
        raw = self._read_raw()
        if raw != self._last_raw:
            self._last_raw = raw
            self._last_change = now_ms
        if time.ticks_diff(now_ms, self._last_change) >= DEBOUNCE_MS:
            if raw != self._stable:
                self._stable = raw
                if self._stable == 1:
                    self.exit_watch_started = None
                    if not self.occupied:
                        self.occupied = True
                        self.time_in = time.time()
                        on_slot_occupied(self)
                else:
                    self.exit_watch_started = now_ms
        if self.exit_watch_started is not None:
            if time.ticks_diff(now_ms, self.exit_watch_started) >= EXIT_GRACE_MS:
                if self.occupied:
                    self.occupied = False
                    self.exit_watch_started = None
                    on_slot_exit(self, time.time())

slots = [Slot(IR_PINS[i], f"S{i+1}") for i in range(3)]
active_tickets = {}

def lowest_free_id():
    used = {s.assigned_id for s in slots if s.assigned_id is not None}
    for i in (1, 2, 3):
        if i not in used:
            return i
    return None

def free_slot_names():
    return [s.name for s in slots if not s.occupied]

def on_slot_occupied(slot):
    if slot.assigned_id is None:
        tid = lowest_free_id()
        if tid is not None:
            slot.assigned_id = tid
            ticket = Ticket(tid, slot.name, slot.time_in)
            active_tickets[tid] = ticket
            print("PARK:", slot.name, "ID", tid)
            lcd_show_temp(f"{slot.name} OCCUPIED")
    else:
        lcd_show_temp(f"{slot.name} OCCUPIED")

def on_slot_exit(slot, t_out):
    tid = slot.assigned_id
    if tid is not None and tid in active_tickets:
        ticket = active_tickets[tid]
        duration_sec, fee = ticket.close(t_out)
        slot.last_duration = duration_sec
        slot.last_fee = fee
        print(f"EXIT: {slot.name} ID {tid} duration {duration_sec}s fee ${fee}")
        lcd_show_temp(f"{slot.name} FREE")
        send_telegram_receipt(ticket)
        del active_tickets[tid]
    slot.assigned_id = None
    slot.time_in = None

# ---------- WEB SERVER (PORT 81) ----------
# <<< WEB SERVER PORT 81 >>>
def web_page():
    gate_status_text = "OPEN" if gate_is_open() else "CLOSED"
    html = """<!DOCTYPE html>
<html><head>
<title>Smart Parking System</title>
<meta http-equiv='refresh' content='3'>
<style>
body{font-family:Arial;text-align:center;background:#fafafa;}
table{margin:auto;border-collapse:collapse;width:70%%;}
td,th{border:1px solid #555;padding:6px;}
th{background:#333;color:#fff;}
.free{color:green;}
.occ{color:red;}
h3{margin-top:6px;}
</style></head><body>
<h2>Smart Parking System</h2>
<h3>Gate Status: %s</h3>
<table><tr><th>Slot</th><th>Status</th><th>Duration</th><th>Fee</th></tr>
""" % gate_status_text
    for s in slots:
        if s.occupied:
            dur = int(time.time() - s.time_in)
            fee = "Free" if dur < 60 else f"${round(math.ceil(dur/60)*PRICE_PER_MIN,2)}"
            html += f"<tr><td>{s.name}</td><td class='occ'>Occupied</td><td>{dur}s</td><td>{fee}</td></tr>"
        else:
            dur = f"{s.last_duration}s" if s.last_duration else "-"
            fee = f"${s.last_fee}" if s.last_fee else "-"
            html += f"<tr><td>{s.name}</td><td class='free'>Free</td><td>{dur}</td><td>{fee}</td></tr>"
    html += "</table><p>Last updated: %s</p></body></html>" % time.strftime("%H:%M:%S", time.localtime())
    return html

def start_web_server(ip):
    try:
        addr = socket.getaddrinfo("0.0.0.0", 81)[0][-1]  # bind all interfaces on port 81
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(addr)
        s.listen(2)
        print("Web server running at: http://%s:%d" % (ip, 81))
    except OSError as e:
        if e.args and e.args[0] == 112:
            print("Port 81 already in use, skipping web server start.")
            return
        else:
            raise

    while True:
        try:
            cl, _ = s.accept()
            cl.settimeout(2)
            _ = cl.recv(1024)  # read request
            response = web_page()
            cl.send(b"HTTP/1.1 200 OK\r\n")
            cl.send(b"Content-Type: text/html\r\n")
            cl.send(b"Connection: close\r\n\r\n")
            cl.send(response)
            cl.close()
        except Exception:
            try:
                cl.close()
            except:
                pass

# ---------- MAIN LOOP ----------
def main():
    ip = connect_wifi()
    gate_close()
    lcd_show_compact()

    import _thread
    _thread.start_new_thread(start_web_server, (ip,))
    print("Web interface started!")

    last_lcd = 0

    while True:
        now_ms = time.ticks_ms()

        # Update slot states (debounced)
        for s in slots:
            s.update(now_ms)

        # MODE 1: gate follows ultrasonic (open when near, close when clear), only if at least one slot free
        if any(not s.occupied for s in slots):
            if car_at_gate():
                gate_open()
            else:
                gate_close()
        else:
            gate_close()  # FULL -> force closed

        # Refresh LCD at interval
        if time.ticks_diff(now_ms, last_lcd) >= LCD_REFRESH_MS:
            lcd_show_compact()
            last_lcd = now_ms

        time.sleep_ms(40)

if __name__ == "__main__":
    main()
