import argparse
import serial
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2
from textual import work
from textual.app import App, ComposeResult
from textual.worker import get_current_worker
from textual.containers import Container, Horizontal, Vertical
from textual.widgets import Header, Footer, DataTable
from textual.widgets import TabbedContent, TabPane, RichLog
from textual.widgets import Button, SelectionList
from textual.widgets.selection_list import Selection
from rich.text import Text

DEBUG = False

apm_copter_mode = ('Stabilize',    'Acro',            'AltHold',  'Auto',   'Guided',
                   'Loiter',       'RTL',             'Circle',   '???',    'Land',
                   '???',          'Drift',           '???',      'Sport',  'Flip',
                   'AutoTune',     'PosHold',         'Brake',    'Throw',  'Avoid_ADSB',
                   'Guided_NoGPS', 'Smart_RTL',       'FlowHold', 'Follow', 'ZigZag',
                   'SystemID',     'Heli_Autorotate', 'Auto RTL', 'Turtle')

all_sev = ('[white on #8D0638] EMER [/] ',
           '[white on #8D0638] ALERT [/]',
           '[white on #8D0638] CRIT [/] ',
           '[white on #8D0638] ERROR [/]', 
           '[white on #B36800] WARN [/] ',
           '[white on #B36800] NOTE [/] ',
                    '[reverse] INFO [/] ',
                    '[reverse] DEBUG [/]',
            '[reverse #575757] APP [/]  ',
            '       ')

# {com_ports:[drone_id]}
telems: dict[mavutil.mavserial, list[int]] = {}
queue_tx = []

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
        act_cont = Container(id='container_actions', classes='big_block')
        act_cont.border_title = 'ACTIONS'
        drone_select_list = SelectionList(id='sel_drones', compact=False)

        yield Header()
        yield Footer()
        with Horizontal():
            with Vertical():
                yield table
                with tabs:
                    with TabPane("All", id='tab_all'):
                        yield RichLog(id='textlog_all')
            with act_cont:
                yield Button('Start mission', variant="primary")
                yield drone_select_list

    def on_mount(self) -> None:
        self.uart_rx()

    def action_toggle_dark(self) -> None:
        self.theme = (
            'textual-dark' if self.theme == 'textual-light' else 'textual-light'
        )

    def print_textlog(self, text: str, severity=None, id=None):
        prefix = time.strftime('[%H:%M:%S] ', time.localtime())

        if severity is None:
            severity = -1
        prefix += f'{all_sev[severity]} '
        if id:
            textlog = self.query_one(f'#textlog_{id}', RichLog)
            textlog.write(Text.from_markup(f'{prefix}{text}'))
            prefix += f'[Drone {id}] '
        textlog = self.query_one('#textlog_all', RichLog)
        textlog.write(Text.from_markup(f'{prefix}{text}'))

    def new_drone(self, id, telem):
        # Строка в таблице
        table = self.query_one('#table_log', DataTable)
        table.add_row(*('-', '-', '-', '-', '-', '-'), key = f'row_{id}', label = f'Drone {id}')
        # Вкладка STATUSTEXT
        new_textlog = RichLog(id=f'textlog_{id}')
        new_tab = TabPane(f'Drone {id}', new_textlog, id=f'tab_{id}')
        tabs = self.query_one('#tabbed_textlog', TabbedContent)
        tabs.add_pane(new_tab)
        # Список во вкладке ACTIONS
        sel_list = self.query_one('#sel_drones', SelectionList)
        # TODO: Убрать цикл после отладки
        sel_list.add_option(Selection(f'Drone {id}', id, id=f'sel_{id}'))
        # Добавление ID в словарь
        telems[telem].append(id)

    def find_telem(self, id):
        return next((port for port, ids in telems.items() if id in ids), None)
    
    def mav_request_log(self, id):
        telem = self.find_telem(id)
        if not telem:
            return False
        # SYS_STATUS
        self.call_from_thread(self.print_textlog, 'SET_MESSAGE_INTERVAL SYS_STATUS', -2, id)
        msg = telem.mav.command_long_encode(id, mav2.MAV_COMP_ID_AUTOPILOT1,  # Target component ID
                                            mav2.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                                            0,  # Confirmation
                                            mav2.MAVLINK_MSG_ID_SYS_STATUS,  # param1: Message ID to be streamed
                                            1000000, # param2: Interval in microseconds
                                            0, 0, 0, 0, 0)
        telem.mav.send(msg)
        # GLOBAL_POSITION_INT
        self.call_from_thread(self.print_textlog, 'SET_MESSAGE_INTERVAL GLOBAL_POSITION_INT', -2, id)
        msg = telem.mav.command_long_encode(id, mav2.MAV_COMP_ID_AUTOPILOT1,  # Target component ID
                                            mav2.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                                            0,  # Confirmation
                                            mav2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # param1: Message ID to be streamed
                                            1000000, # param2: Interval in microseconds
                                            0, 0, 0, 0, 0)
        telem.mav.send(msg)
        # msg = telem.mav.command_long_encode(id, mav2.MAV_COMP_ID_AUTOPILOT1,  # Target component ID
        #                                     mav2.MAV_CMD_IMAGE_START_CAPTURE,  # ID of command to send
        #                                     0,  # Confirmation
        #                                     0, # ID камеры
        #                                     0.1, # Интервал в сек
        #                                     0, # Кол-во фото
        #                                     0, 0, 0, 0)
        # telem.mav.send(msg)
        
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
                        self.call_from_thread(self.print_textlog, f'{telem.port.name} Disconnected', 8)
                        sel_list = self.query_one('#sel_drones', SelectionList)
                        for id in telems[telem]:
                            sel_drone = sel_list.get_option(f'sel_{id}')
                            sel_drone.disabled = True
                            sel_list.deselect(sel_drone)
                            self.call_from_thread(self.print_textlog, f'Disconnected', 8, id)
                            # Т.к. рендер SelectionList не обновляется, если deselect не сработал (пункт не выбран изначально)
                            sel_list.refresh()
                        del telems[telem]
                        if len(telems) == 0: # Иначе у программы инпут лаг
                            return
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
                                self.call_from_thread(self.new_drone, rx_sysid, telem)
                                self.mav_request_log(rx_sysid)
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
                        rx_msg_id = rx_msg.get_msgId()
                        if rx_msg_id == mav2.MAVLINK_MSG_ID_SYS_STATUS:
                            rx_stat: mav2.MAVLink_sys_status_message = rx_msg
                            # Обновление Batt Volt
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'bat_col', f'{(rx_stat.voltage_battery / 1000):.2f}', update_width=True)
                        elif rx_msg_id == mav2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                            rx_gpos: mav2.MAVLink_global_position_int_message = rx_msg
                            # Обновление Latitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'lat_col', f'{rx_gpos.lat / 10000000}', update_width=True)
                            # Обновление Longitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'lon_col', f'{rx_gpos.lon / 10000000}', update_width=True)
                            # Обновление Altitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'alt_col', f'{(rx_gpos.relative_alt / 1000):.2f}', update_width=True)
                        elif rx_msg_id == mav2.MAVLINK_MSG_ID_STATUSTEXT:
                            rx_text: mav2.MAVLink_statustext_message = rx_msg
                            # TODO: Как-то проверить текст из нескольких чанков
                            self.call_from_thread(self.print_textlog, rx_text.text, rx_text.severity, rx_sysid)
                        elif rx_msg_id == mav2.MAVLINK_MSG_ID_COMMAND_ACK:
                            rx_cmd_ack: mav2.MAVLink_command_ack_message = rx_msg
                            self.call_from_thread(self.print_textlog, f'ACK: cmd: {rx_cmd_ack.command} result: {rx_cmd_ack.result}', None, rx_sysid)
                        elif rx_msg_id == mav2.MAVLINK_MSG_ID_PARAM_VALUE:
                            rx_param_val: mav2.MAVLink_param_value_message = rx_msg
                            self.call_from_thread(self.print_textlog, f'PARAM_VALUE: id: {rx_param_val.param_id}', None, rx_sysid)
                        else:
                            self.call_from_thread(self.print_textlog, f'Message {rx_msg.get_type()} received', None, rx_sysid)
            except Exception as e:
                self.call_from_thread(self.print_textlog, f'App Failure ({e})', 0)
                return

    @work(exclusive=True, thread=True)
    def uart_tx(self):
        pass

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
