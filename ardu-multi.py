# import threading
import time
from textual import work
from textual.app import App, ComposeResult
from textual.containers import Vertical
from textual.widgets import Header, Footer, DataTable, TabbedContent, TabPane, RichLog
from textual.worker import get_current_worker
from rich.text import Text

# TODO: Переделать под MAVLink
#       Словарь со словарями
#       {"id дрона":{id сообщения:сообщение}}
mav_logs = {"1":["STAB", "ARM", "22.4", "14.025167", "28.821908", 20], 
            "2":["AUTO", "DISARM", "14.0", "52.675797", "17.247457", 33],
            "3":["LOIT", "ARM", "12.3", "-47.679444", "-9.585835", 432]}
# TODO: Просто STATUSTEXT В mav_logs
mav_s_text = ["Lorem ipsum dolor sit amet,", 
              "consectetur adipiscing elit,", 
              "sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.",
              "Ut enim ad minim veniam,",
              "quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat.",
              "Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.",
              "Excepteur sint occaecat cupidatat non proident",
              "sunt in culpa qui officia deserunt mollit anim id est laborum."]

class ArduMultiApp(App):
    CSS_PATH = "style.tcss"
    TITLE = "Multi ArduPilot Drone Monitor"
    BINDINGS = [("q,Q,й,Й", "quit", "Quit"),
                ("d", "toggle_dark", "Toggle dark mode")]

    def compose(self) -> ComposeResult:
        table = DataTable(cursor_type = "none", zebra_stripes = True, id='table_log', classes="big_block")
        table.border_title = "TELEMETRY"
        table.add_columns(("Mode", "mode_col"),
                          ("Arm", "arm_col"),
                          ("Batt Volt", "Bat_col"),
                          ("Latitude", "lat_col"),
                          ("Longitude", "lon_col"),
                          ("Height", "height_col"))
        tabs = TabbedContent(id='tabbed_textlog', classes="big_block")
        tabs.border_title = "STATUSTEXT"

        yield Header()
        yield Footer()
        with Vertical():
            yield table
            yield tabs

    def on_mount(self) -> None:
        self.uart_rx()

    def action_toggle_dark(self) -> None:
        self.theme = (
            "textual-dark" if self.theme == "textual-light" else "textual-light"
        )

    # ------------
    # Чтение UART
    # ------------
    @work(exclusive=True, thread=True)
    def uart_rx(self):
        worker = get_current_worker()
        table = self.query_one('#table_log', DataTable)
        # Создание новых строк и вкладок ВРЕМЕННО ТАК
        for id, data_log in mav_logs.items():
            self.call_from_thread(table.add_row, *data_log, key = f"row_{id}", label = f"Drone {id}")
            self.call_from_thread(self.new_statustext, id)

        # Обновление данных
        i = 0
        while True:
            if worker.is_cancelled:
                return
            mav_logs["1"][5] += 1
            # Таблица
            self.call_from_thread(table.update_cell, f"row_{1}", "height_col", mav_logs["1"][5])
            # Statustext
            textlog = self.query_one(f"#textlog_{1}", RichLog)
            self.call_from_thread(textlog.write, Text(f"• {mav_s_text[i]}", style="green"))
            i = (i+1) % len(mav_s_text)
            # TODO: Избавиться от Sleep, он мешает!!!
            time.sleep(1)

    def new_statustext(self, id):
        new_tabs = self.query_one('#tabbed_textlog', TabbedContent)
        new_textlog = RichLog(id=f"textlog_{id}")
        pane = TabPane(f"TAB {id}", new_textlog, id=f"tab_{id}")
        new_tabs.add_pane(pane)


if __name__ == "__main__":
    app = ArduMultiApp()
    app.run()
