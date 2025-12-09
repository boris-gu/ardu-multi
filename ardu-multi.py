from dataclasses import dataclass, field
from queue import Queue, Empty
import copy
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
sec2ns = 1000000000

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
telems: dict[mavutil.mavserial, Telem_data] = {}

@dataclass
class Telem_data:
    ids: list[int] = field(default_factory=list)
    fifo_tx: Queue = field(default_factory=Queue)
    fifo_rx_all: dict[str, Queue] = field(default_factory=dict)


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
        for telem in telems:
            self.uart_tx(telem)

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

    def find_telem(self, id):
        return next((port for port, port_data in telems.items() if id in port_data.ids), None)

    # ------------
    # Чтение UART
    # ------------
    @work(thread=True)
    def uart_rx(self):
        worker = get_current_worker()
        table = self.query_one('#table_log', DataTable)
        # Обновление данных
        if DEBUG:
            for i in range(len(all_sev)):
                self.call_from_thread(self.print_textlog, 'Lorem ipsum dolor', i)
        while worker.is_running:
            try:
                for telem in telems:
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
                    for fifo_rx in telems[telem].fifo_rx_all.values():
                        fifo_rx.put(msg_rx)
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
                            if all(rx_sysid not in telem_data.ids for telem_data in telems.values()):
                                self.call_from_thread(self.task_add_new_drone, rx_sysid, telem)
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
                    elif any(rx_sysid in telem_data.ids for telem_data in telems.values()): # TODO: Не уверен
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
                            self.call_from_thread(self.print_textlog, f'Received ACK: cmd: {rx_cmd_ack.command} result: {rx_cmd_ack.result}', None, rx_sysid)
                        else:
                            self.call_from_thread(self.print_textlog, f'Received Message {msg_rx.get_type()}', None, rx_sysid)
            except Exception as e:
                self.call_from_thread(self.print_textlog, f'App Failure ({e})', 0)
                return

    # Отправка сообщений и Protocol Heartbeat
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

    @work(thread=True)
    def task_add_new_drone(self, id, telem):
        # Строка в таблице
        table = self.query_one('#table_log', DataTable)
        self.call_from_thread(table.add_row, *('-', '-', '-', '-', '-', '-'), key = f'row_{id}', label = f'Drone {id}')
        # Вкладка STATUSTEXT
        new_textlog = RichLog(id=f'textlog_{id}')
        new_tab = TabPane(f'Drone {id}', new_textlog, id=f'tab_{id}')
        tabs = self.query_one('#tabbed_textlog', TabbedContent)
        self.call_from_thread(tabs.add_pane, new_tab)
        # Список во вкладке ACTIONS
        sel_list = self.query_one('#sel_drones', SelectionList)
        self.call_from_thread(sel_list.add_option, Selection(f'Drone {id}', id, id=f'sel_{id}'))
        # Добавление ID в словарь
        telems[telem].ids.append(id)
        # Запрос логов
        # SYS_STATUS
        self.call_from_thread(self.print_textlog, 'SET_MESSAGE_INTERVAL SYS_STATUS', -2, id)
        self.protocol_command_long(id, mav2.MAV_COMP_ID_AUTOPILOT1,  # Target component ID
                                   mav2.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                                   0,  # Confirmation
                                   mav2.MAVLINK_MSG_ID_SYS_STATUS,  # param1: Message ID to be streamed
                                   1000000, # param2: Interval in microseconds
                                   0, 0, 0, 0, 0)
        # GLOBAL_POSITION_INT
        self.call_from_thread(self.print_textlog, 'SET_MESSAGE_INTERVAL GLOBAL_POSITION_INT', -2, id)
        self.protocol_command_long(id, mav2.MAV_COMP_ID_AUTOPILOT1,
                                   mav2.MAV_CMD_SET_MESSAGE_INTERVAL,
                                   0,  # Confirmation
                                   mav2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                   1000000,
                                   0, 0, 0, 0, 0)
        
    @work(thread=True)
    def protocol_command_long(self,
                              target_system: int,
                              target_component: int,
                              command: int,
                              confirmation: int,
                              param1: float,
                              param2: float,
                              param3: float,
                              param4: float,
                              param5: float,
                              param6: float,
                              param7: float):
        worker = get_current_worker()
        telem = self.find_telem(target_system)
        mav_tx: mav2.MAVLink = telem.mav
        this_fifo = Queue() # Создаем FIFO
        # Проверяем, есть ли уже эта очередь в словаре (уже вызвали у этого дрона)
        # Есть - ждем освобождения
        # Нет - добавляем очередь в словарь всех FIFO
        while telems[telem].fifo_rx_all.setdefault(f'command_long_{target_system}', this_fifo) is not this_fifo:
            time.sleep(0.1)
        # Отправляем сообщение с командой
        msg_tx = mav_tx.command_long_encode(target_system, target_component, command, confirmation,
                                            param1, param2, param3, param4, param5, param6, param7)
        for _ in range(3):
            telems[telem].fifo_tx.put(msg_tx) # Отправляем сообщение с командой
            # Ждем ответ
            start_wait = time.time_ns()
            while  (worker.is_running and
                    (telem in telems) and
                    (time.time_ns() - start_wait <= sec2ns)):
                try:
                    msg_rx: mav2.MAVLink_message = this_fifo.get(timeout=1)
                    rx_sysid = msg_rx.get_srcSystem()
                    # Получили ответ от нужного дрона
                    if rx_sysid == target_system and msg_rx.get_msgId() == mav2.MAVLINK_MSG_ID_COMMAND_ACK:
                        rx_cmd_ack: mav2.MAVLink_command_ack_message = msg_rx
                        # Ответ на нужную команду
                        if rx_cmd_ack.command == command:
                            if rx_cmd_ack.result == mav2.MAV_RESULT_IN_PROGRESS:
                                start_wait = time.time_ns()
                            else:
                                del telems[telem].fifo_rx_all[f'command_long_{target_system}']
                                return
                except Empty:
                    pass
        del telems[telem].fifo_rx_all[f'command_long_{target_system}']


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
            telems[mavutil.mavlink_connection(port, baud)] = Telem_data()
        app = ArduMultiApp()
        app.run()
    except Exception as e:
        print(parser.format_usage(), end='')
        print(f'ERROR: {e}')
    finally:
        pass
