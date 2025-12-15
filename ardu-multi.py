from dataclasses import dataclass, field
from queue import Queue, Empty
import threading
import argparse
import serial
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2
from textual import work, on
from textual.app import App, ComposeResult
from textual.worker import get_current_worker
from textual.containers import Container, Horizontal, Vertical
from textual.widgets import Header, Footer, DataTable
from textual.widgets import TabbedContent, TabPane, RichLog
from textual.widgets import Button, SelectionList
from textual.widgets.selection_list import Selection
from rich.text import Text

sec2ns = 1000000000

apm_arm_status = {0:'DISARM', mav2.MAV_MODE_FLAG_SAFETY_ARMED:'ARM'}

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

@dataclass
class Telem_data:
    ids: list[int] = field(default_factory=list)
    fifo_tx: Queue = field(default_factory=Queue)

@dataclass
class Drone_data:
    heartbeat: mav2.MAVLink_heartbeat_message | None = None
    cmd_ack: mav2.MAVLink_command_ack_message | None = None
    prot_cmd_lock: threading.Lock = field(default_factory=threading.Lock)

# {com_ports:[drone_id]}
telems: dict[mavutil.mavserial, Telem_data] = {}
drones: dict[int, Drone_data] = {}


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
                yield Button('Start Photo', variant="primary", id='btn_start_photo')
                yield Button('Stop Photo', variant="primary", id='btn_stop_photo')
                yield drone_select_list

    def on_mount(self) -> None:
        self.uart_rx()
        for telem in telems:
            self.uart_tx(telem)

    def action_toggle_dark(self) -> None:
        self.theme = ('textual-dark' if self.theme == 'textual-light' else 'textual-light')
    
    @on(Button.Pressed, "#btn_start_photo")  
    def btn_start_photo(self):
        self.task_start_stop_photo(True, 2)

    @on(Button.Pressed, "#btn_stop_photo")  
    def btn_stop_photo(self):
        self.task_start_stop_photo()

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

    def find_telem(self, id):
        return next((port for port, port_data in telems.items() if id in port_data.ids), None)

    # ------------
    # Чтение UART
    # ------------
    @work(thread=True)
    def uart_rx(self):
        worker = get_current_worker()
        table = self.query_one('#table_log', DataTable)
        # if args.debug:
        #     for i in range(len(all_sev)):
        #         self.call_from_thread(self.print_textlog, 'Lorem ipsum dolor', i)
        # Обновление данных
        while worker.is_running:
            try:
                for telem in telems:
                    # TODO: Переписать отключение телеметрии!!!
                    try: # Ловим, если порт был отключен
                        msg_rx = telem.recv_match(blocking=False)
                    except (serial.SerialException, serial.SerialTimeoutException):
                        self.call_from_thread(self.print_textlog, f'{telem.port.name} Disconnected', 8)
                        sel_list = self.query_one('#sel_drones', SelectionList)
                        for id in telems[telem].ids:
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
                    if msg_rx is None:
                        continue
                    msg_rx: mav2.MAVLink_message
                    rx_sysid = msg_rx.get_srcSystem()
                    # Проверка на HEARTBEAT отличается от других сообщений т.к. по нему находим новые дроны
                    if msg_rx.get_msgId() == mav2.MAVLINK_MSG_ID_HEARTBEAT:
                        rx_heartbeat: mav2.MAVLink_heartbeat_message = msg_rx
                        if (rx_heartbeat.autopilot == mav2.MAV_AUTOPILOT_ARDUPILOTMEGA and
                            (rx_heartbeat.type == mav2.MAV_TYPE_QUADROTOR or
                            rx_heartbeat.type == mav2.MAV_TYPE_HEXAROTOR or
                            rx_heartbeat.type == mav2.MAV_TYPE_OCTOROTOR)):
                            # Получили HEARTBEAT от нового дрона, ищем среди списков id у каждого com порта
                            if rx_sysid not in drones:
                                self.call_from_thread(self.task_add_drone, rx_sysid, telem)
                            drones[rx_sysid].heartbeat = rx_heartbeat
                            # Обновление Arm
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'arm_col',
                                                  apm_arm_status[rx_heartbeat.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED],
                                                  update_width=True)
                            # Обновление Mode
                            copter_mode_str = mav2.enums['COPTER_MODE'][rx_heartbeat.custom_mode].name.replace('COPTER_MODE_', '')
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'mode_col', copter_mode_str, update_width=True)
                    elif rx_sysid in drones:
                        msg_rx_id = msg_rx.get_msgId()
                        if msg_rx_id == mav2.MAVLINK_MSG_ID_SYS_STATUS:
                            rx_stat: mav2.MAVLink_sys_status_message = msg_rx
                            # Обновление Batt Volt
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'bat_col', f'{(rx_stat.voltage_battery / 1000):.2f}', update_width=True)
                        elif msg_rx_id == mav2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                            rx_gpos: mav2.MAVLink_global_position_int_message = msg_rx
                            # Обновление Latitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'lat_col', f'{rx_gpos.lat / 10000000}', update_width=True)
                            # Обновление Longitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'lon_col', f'{rx_gpos.lon / 10000000}', update_width=True)
                            # Обновление Altitude
                            self.call_from_thread(table.update_cell, f'row_{rx_sysid}', 'alt_col', f'{(rx_gpos.relative_alt / 1000):.2f}', update_width=True)
                        elif msg_rx_id == mav2.MAVLINK_MSG_ID_STATUSTEXT:
                            rx_text: mav2.MAVLink_statustext_message = msg_rx
                            # TODO: Как-то проверить текст из нескольких чанков
                            self.call_from_thread(self.print_textlog, rx_text.text, rx_text.severity, rx_sysid)
                        elif msg_rx_id == mav2.MAVLINK_MSG_ID_COMMAND_ACK:
                            rx_cmd_ack: mav2.MAVLink_command_ack_message = msg_rx
                            drones[rx_sysid].cmd_ack = rx_cmd_ack
                            if args.debug:
                                cmd_str = mav2.enums['MAV_CMD'][rx_cmd_ack.command].name.replace('MAV_CMD_', '')
                                cmd_result_str = mav2.enums['MAV_RESULT'][rx_cmd_ack.result].name.replace('MAV_RESULT_', '')
                                self.call_from_thread(self.print_textlog, f'Received ACK: cmd: [b]{cmd_str}[/] result: [b]{cmd_result_str}[/]', None, rx_sysid)
                        else:
                            if args.debug:
                                self.call_from_thread(self.print_textlog, f'Received [b]{msg_rx.get_type()}[/]', None, rx_sysid)
            except Exception as e:
                self.call_from_thread(self.print_textlog, f'App Failure ({e})', 0)
                return

    # ----------------------------------------
    # Отправка сообщений и Protocol Heartbeat
    # ----------------------------------------
    @work(thread=True)
    def uart_tx(self, telem):
        worker = get_current_worker()
        last_hb = time.time_ns()
        while worker.is_running and (telem in telems):
            mav_tx: mav2.MAVLink = telem.mav
            if time.time_ns() - last_hb >= sec2ns:
                last_hb += sec2ns
                mav_tx.heartbeat_send(mav2.MAV_TYPE_GCS, mav2.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            try:
                mav_tx.send(telems[telem].fifo_tx.get(block=False))
            except Empty: # XXX: Возможно не нужно
                pass
            time.sleep(0.05) # Иначе виснет рендер

    def task_add_drone(self, id, telem):
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
        sel_list.add_option(Selection(f'Drone {id}', id, id=f'sel_{id}'))
        # Добавление ID в словарь телеметрии и в словарь дронов
        telems[telem].ids.append(id)
        drones[id] = Drone_data()
        # Запрос логов
        # SYS_STATUS
        if args.debug:
            self.print_textlog('Send [b]SET_MESSAGE_INTERVAL[/] msg: [b]SYS_STATUS[/]', -2, id)
        self.protocol_command_long(id, mav2.MAV_COMP_ID_AUTOPILOT1,  # Target component ID
                                   mav2.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                                   0,  # Confirmation
                                   mav2.MAVLINK_MSG_ID_SYS_STATUS,  # param1: Message ID to be streamed
                                   1000000) # param2: Interval in microseconds
        # GLOBAL_POSITION_INT
        if args.debug:
            self.print_textlog('Send [b]SET_MESSAGE_INTERVAL[/] msg: [b]GLOBAL_POSITION_INT[/]', -2, id)
        self.protocol_command_long(id, mav2.MAV_COMP_ID_AUTOPILOT1,
                                   mav2.MAV_CMD_SET_MESSAGE_INTERVAL,
                                   0,  # Confirmation
                                   mav2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                   1000000)

    def task_remove_telem(self, telem):
        # TODO: Написать
        pass

    @work(thread=True)
    def task_start_stop_photo(self, is_start: int = False, interval: float = 0):
        task_str = 'Start' if is_start else 'Stop'
        task_cmd = mav2.MAV_CMD_IMAGE_START_CAPTURE if is_start else mav2.MAV_CMD_IMAGE_STOP_CAPTURE
        sel_list = self.query_one('#sel_drones', SelectionList)
        ids = sel_list.selected
        if len(ids) == 0:
            self.call_from_thread(self.print_textlog, f'[b]FAILED[/] {task_str} Photo: [b]No drone selected[/]', -2)
            return
        self.call_from_thread(self.print_textlog, f'[b]{task_str.upper()} PHOTO[/]', -2)
        for id in ids:
            self.protocol_command_long(id, mav2.MAV_COMP_ID_AUTOPILOT1,  # Target component ID
                                    task_cmd,  # ID of command to send
                                    0,  # Confirmation
                                    0,  # Cam id (0 - all)
                                    interval)

    @work(thread=True)
    def protocol_command_long(self,
                              target_system: int,
                              target_component: int,
                              command: int,
                              confirmation: int = 0,
                              param1: float = 0,
                              param2: float = 0,
                              param3: float = 0,
                              param4: float = 0,
                              param5: float = 0,
                              param6: float = 0,
                              param7: float = 0,
                              attempts_сount: int = 3):
        worker = get_current_worker()
        wait_time = sec2ns
        telem = self.find_telem(target_system)
        mav_tx: mav2.MAVLink = telem.mav
        msg_tx = mav_tx.command_long_encode(target_system, target_component, command, confirmation,
                                            param1, param2, param3, param4, param5, param6, param7)
        with drones[target_system].prot_cmd_lock:
            for _ in range(attempts_сount):
                telems[telem].fifo_tx.put(msg_tx) # Отправляем сообщение с командой
                # Ждем ответ
                start_wait = time.time_ns()
                while  (worker.is_running and
                        (telem in telems) and
                        (time.time_ns() - start_wait <= wait_time)):
                    if ((drones[target_system].cmd_ack is not None) and
                        drones[target_system].cmd_ack.command == command):
                        if drones[target_system].cmd_ack.result == mav2.MAV_RESULT_IN_PROGRESS:
                            drones[target_system].cmd_ack = None
                            start_wait = time.time_ns()
                        else:
                            drones[target_system].cmd_ack = None
                            return
                    time.sleep(0.2)
            drones[target_system].cmd_ack = None


def com_parse(arg):
    try:
        port, baudrate = arg.split(':')
        return (port.upper(), int(baudrate))
    except:
        raise argparse.ArgumentTypeError(f'[{arg}] use format PORT:BAUDRATE')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug',
                        action='store_true',
                        help='Enable debug output')
    parser.add_argument('-p', '--port',
                        action='append',
                        type=com_parse,
                        required=True,
                        help='COM port with MAVLink telemetry',
                        metavar='PORT:BAUDRATE')
    args = parser.parse_args()
    # Удаление повторений, если есть (Уже не нужно, есть словарь)
    ports_dict = dict(args.port) if args.port else {}

    try:
        for port, baud in ports_dict.items():
            telems[mavutil.mavlink_connection(port, baud)] = Telem_data()
        app = ArduMultiApp()
        app.run()
    except Exception as e:
        print(parser.format_usage(), end='')
        print(f'ERROR: {e}')
    finally:
        pass
