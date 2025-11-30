import argparse
import serial
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2
from textual import work
from textual.app import App, ComposeResult
from textual.containers import Vertical
from textual.widgets import Header, Footer, DataTable, TabbedContent, TabPane, RichLog
from textual.worker import get_current_worker
from rich.text import Text

DEBUG = False

apm_copter_mode = ('Stabilize',    'Acro',            'AltHold',  'Auto',   'Guided',
                   'Loiter',       'RTL',             'Circle',   '???',    'Land',
                   '???',          'Drift',           '???',      'Sport',  'Flip',
                   'AutoTune',     'PosHold',         'Brake',    'Throw',  'Avoid_ADSB',
                   'Guided_NoGPS', 'Smart_RTL',       'FlowHold', 'Follow', 'ZigZag',
                   'SystemID',     'Heli_Autorotate', 'Auto RTL', 'Turtle')

all_sev = (('EMERGENCY', 'white on #8D0638'),
           ('ALERT',     'white on #8D0638'),
           ('CRITICAL',  'white on #8D0638'),
           ('ERROR',     'white on #8D0638'),
           ('WARNING',   'white on #B36800'),
           ('NOTICE',    'white on #B36800'),
           ('INFO',      'reverse'),
           ('DEBUG',     'reverse'),
           ('APP DEBUG', 'reverse'))

# {com_ports:[drone_id]}
telems: dict[mavutil.mavserial, list[int]] = {}
# telems: list[mavutil.mavserial] = []
mav_logs = {}

class ArduMultiApp(App):
    CSS_PATH = 'style.tcss'
    TITLE = 'Multi ArduPilot Drone Monitor'
    BINDINGS = [('q,Q,й,Й', 'quit', 'Quit'),
                ('d,D,в,В', 'toggle_dark', 'Toggle dark mode')]

    def compose(self) -> ComposeResult:
        table = DataTable(cursor_type = 'none', zebra_stripes = True, id='table_log', classes='big_block')
        table.border_title = 'TELEMETRY'
        table.add_columns(('Arm', 'arm_col'),
                          ('Mode', 'mode_col'),
                          ('Batt Volt', 'bat_col'),
                          ('Latitude', 'lat_col'),
                          ('Longitude', 'lon_col'),
                          ('Altitude', 'alt_col'))
        tabs = TabbedContent(id='tabbed_textlog', classes='big_block')
        tabs.border_title = 'STATUSTEXT'

        yield Header()
        yield Footer()
        with Vertical():
            yield table
            with tabs:
                with TabPane("All", id='tab_all'):
                    yield RichLog(id='textlog_all')

    def on_mount(self) -> None:
        self.uart_rx()

    def action_toggle_dark(self) -> None:
        self.theme = (
            'textual-dark' if self.theme == 'textual-light' else 'textual-light'
        )

    # ------------
    # Чтение UART
    # ------------
    @work(exclusive=True, thread=True)
    def uart_rx(self):
        worker = get_current_worker()
        table = self.query_one('#table_log', DataTable)
        # Обновление данных
        if DEBUG:
            for i in range(len(all_sev)):
                self.call_from_thread(self.print_textlog, 'Lorem ipsum dolor', i)
        while True:
            if worker.is_cancelled:
                return
            try:
                for telem in telems:
                    try: # Ловим, если порт был отключен
                        rx_msg = telem.recv_match(blocking=False)
                    except (serial.SerialException, serial.SerialTimeoutException):
                        self.call_from_thread(self.print_textlog, f'{telem.port.name} Disconnected', 2)
                        for id in telems[telem]:
                            self.call_from_thread(self.print_textlog, f'Disconnected', 2, id)
                        del telems[telem]
                        break
                    if rx_msg is None:
                        continue
                    rx_msg: mav2.MAVLink_message
                    rx_sysid = rx_msg.get_srcSystem()
                    # Проверка на HEARTBEAT отличается от других сообщений т.к. по нему находим новые дроны
                    if rx_msg.get_msgId() == mav2.MAVLINK_MSG_ID_HEARTBEAT:
                        rx_heartbeat: mav2.MAVLink_heartbeat_message = rx_msg
                        if (rx_heartbeat.autopilot == mav2.MAV_AUTOPILOT_ARDUPILOTMEGA and
                            (rx_heartbeat.type == mav2.MAV_TYPE_QUADROTOR or
                            rx_heartbeat.type == mav2.MAV_TYPE_HEXAROTOR or
                            rx_heartbeat.type == mav2.MAV_TYPE_OCTOROTOR)):
                            # Получили HEARTBEAT от нового дрона, ищем среди списков id у каждого com порта
                            if all(rx_sysid not in id_list for id_list in telems.values()):
                                self.call_from_thread(self.new_drone, telem, rx_sysid)
                            # Обновление Arm
                            if rx_heartbeat.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED:
                                self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'arm_col', 'Armed', update_width=True)
                            else:
                                self.call_from_thread(
                                    table.update_cell,
                                    f'row_{rx_sysid}',
                                    'arm_col',
                                    'Disarmed',
                                    update_width=True,
                                )
                            # Обновление Mode
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'mode_col', apm_copter_mode[rx_heartbeat.custom_mode], update_width=True)
                    elif any(rx_sysid in id_list for id_list in telems.values()):
                        if rx_msg.get_msgId() == mav2.MAVLINK_MSG_ID_SYS_STATUS:
                            rx_stat: mav2.MAVLink_sys_status_message = rx_msg
                            # Обновление Batt Volt
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'bat_col', f'{(rx_stat.voltage_battery / 1000):.2f}', update_width=True)
                        if rx_msg.get_msgId() == mav2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                            rx_gpos: mav2.MAVLink_global_position_int_message = rx_msg
                            # Обновление Latitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'lat_col', f'{rx_gpos.lat / 10000000}', update_width=True)
                            # Обновление Longitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'lon_col', f'{rx_gpos.lon / 10000000}', update_width=True)
                            # Обновление Altitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'alt_col', f'{(rx_gpos.relative_alt / 1000):.2f}', update_width=True)
                        elif rx_msg.get_msgId() == mav2.MAVLINK_MSG_ID_STATUSTEXT:
                            rx_text: mav2.MAVLink_statustext_message = rx_msg
                            # TODO: Как-то проверить текст из нескольких чанков
                            self.call_from_thread(self.print_textlog, rx_text.text, rx_text.severity, rx_sysid)
                        elif DEBUG:
                            self.call_from_thread(self.print_textlog, f'Message {rx_msg.get_type()} received', -1, rx_sysid)
            except Exception as e:
                self.call_from_thread(self.print_textlog, f'App Failure ({e})', 0)
                return

    def new_drone(self, telem, id):
        # Строка в таблице
        table = self.query_one('#table_log', DataTable)
        table.add_row(*('-', '-', '-', '-', '-', '-'), key = f'row_{id}', label = f'Drone {id}')
        # Вкладка STATUSTEXT
        new_textlog = RichLog(id=f'textlog_{id}')
        new_tab = TabPane(f'Drone {id}', new_textlog, id=f'tab_{id}')
        tabs = self.query_one('#tabbed_textlog', TabbedContent)
        tabs.add_pane(new_tab)
        # Добавление ID в словарь
        telems[telem].append(id)

    def print_textlog(self, text: str, severity=-1, id=None):
        prefix = time.strftime('[%H:%M:%S]', time.localtime())
        if id:
            textlog = self.query_one(f'#textlog_{id}', RichLog)
            textlog.write(Text.from_markup(f'{prefix} [{all_sev[severity][1]}] {all_sev[severity][0]} [/] {text}'))
            prefix += f' [Drone {id}]'
        textlog = self.query_one('#textlog_all', RichLog)
        textlog.write(Text.from_markup(f'{prefix} [{all_sev[severity][1]}] {all_sev[severity][0]} [/] {text}'))

def com_parse(arg):
    try:
        port, baudrate = arg.split(':')
        return (port.upper(), int(baudrate))
    except:
        raise argparse.ArgumentTypeError(f'[{arg}] use format PORT:BAUDRATE')
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port',
                        type=com_parse,
                        action='append',
                        metavar='PORT:BAUDRATE',
                        required=True)
    args = parser.parse_args()
    # Удаление повторений, если есть (Уже не нужно, есть словарь)
    ports_dict = dict(args.port) if args.port else {}

    try:
        for port, baud in ports_dict.items():
            telems[mavutil.mavlink_connection(port, baud)] = []
        app = ArduMultiApp()
        app.run()
    except Exception as e:
        print(parser.format_usage(), end='')
        print(f'ERROR: {e}')
    finally:
        pass
