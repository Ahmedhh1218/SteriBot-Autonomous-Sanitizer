import sys
import sqlite3
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class DatabaseManager:
    def __init__(self, database_name):
        self.connection = sqlite3.connect(database_name)
        self.cursor = self.connection.cursor()

    def get_user(self, username):
        query = "SELECT * FROM users WHERE username = ?"
        self.cursor.execute(query, (username,))
        user_data = self.cursor.fetchone()
        return user_data if user_data else None

class SplashScreen(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SteriBot Splash Screen")
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setGeometry(0, 0, QApplication.desktop().screenGeometry().width(), QApplication.desktop().screenGeometry().height())
        
        # Set background image
        bg_image = QLabel(self)
        bg_pixmap = QPixmap("3D-Hospital-Environment.png")
        screen_size = QApplication.desktop().screenGeometry().size()
        scaled_bg_pixmap = bg_pixmap.scaled(screen_size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
        bg_image.setPixmap(scaled_bg_pixmap)
        bg_image.setGeometry(0, 0, screen_size.width(), screen_size.height())
        
        # Dark overlay
        dark_overlay = QWidget(self)
        dark_overlay.setStyleSheet("background-color: rgba(0, 0, 0, 150);")
        dark_overlay.setGeometry(0, 0, screen_size.width(), screen_size.height())
        
        # Layout for splash screen content
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        
        # Logo
        logo_label = QLabel()
        logo_pixmap = QPixmap("logo2.jpeg")
        logo_pixmap = logo_pixmap.scaled(200, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # Circle the logo
        circled_pixmap = QPixmap(logo_pixmap.size())
        circled_pixmap.fill(Qt.transparent)
        painter = QPainter(circled_pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        path = QPainterPath()
        path.addEllipse(0, 0, logo_pixmap.width(), logo_pixmap.height())
        painter.setClipPath(path)
        painter.drawPixmap(0, 0, logo_pixmap)
        painter.end()
        
        logo_label.setPixmap(circled_pixmap)
        logo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)
        
        # Slogan
        slogan_label = QLabel("YOUR GUARDIAN OF PURITY IN THE BATTLE OF GERMS")
        slogan_label.setStyleSheet("color: white; font-size: 18px;")
        slogan_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(slogan_label)
        
        # Get Started button
        get_started_button = QPushButton("Get Started")
        get_started_button.setStyleSheet("font-size: 16px; padding: 10px 20px;")
        get_started_button.clicked.connect(self.show_login_screen)
        layout.addWidget(get_started_button)
        
        # Center the content
        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        central_widget.setGeometry(0, 0, screen_size.width(), screen_size.height())
        
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(central_widget)
        main_layout.setAlignment(Qt.AlignCenter)
        self.setLayout(main_layout)
        
    def show_login_screen(self):
        self.accept()

class LoginScreen(QDialog):
    def __init__(self, db_manager):
        super().__init__()
        self.setWindowTitle("Login")
        self.db_manager = db_manager
        self.setWindowFlags(Qt.FramelessWindowHint)  # Remove window frame
        self.setGeometry(0, 0, QApplication.desktop().screenGeometry().width(), QApplication.desktop().screenGeometry().height())  # Set window size to fullscreen

        # Set background image
        bg_image = QLabel(self)
        pixmap = QPixmap("logo.jpeg")
        screen_size = QApplication.desktop().screenGeometry().size()
        scaled_pixmap = pixmap.scaled(screen_size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)

        # Create a semi-transparent pixmap
        transparent_pixmap = QPixmap(scaled_pixmap.size())
        transparent_pixmap.fill(Qt.transparent)
        painter = QPainter(transparent_pixmap)
        painter.setOpacity(0.3)  # Set transparency level for the image
        painter.drawPixmap(0, 0, scaled_pixmap)
        painter.end()

        bg_image.setPixmap(transparent_pixmap)
        bg_image.setGeometry(0, 0, screen_size.width(), screen_size.height())

        # Add dark shade overlay
        dark_overlay = QWidget(self)
        dark_overlay.setStyleSheet("background-color: rgba(0, 0, 0, 150);")  # Semi-transparent black
        dark_overlay.setGeometry(0, 0, screen_size.width(), screen_size.height())

        # Layout for the login form
        form_layout = QVBoxLayout()
        form_layout.setContentsMargins(0, 0, 0, 0)  # Adjust margins as needed
        form_layout.setAlignment(Qt.AlignCenter)  # Center align the widgets

        # Center widget for the form
        form_widget = QWidget()
        form_widget.setLayout(form_layout)
        form_widget.setFixedSize(300, 150)  # Fixed size for the form

        self.username_input = QLineEdit()
        self.username_input.setPlaceholderText("Username")
        form_layout.addWidget(self.username_input)

        self.password_input = QLineEdit()
        self.password_input.setPlaceholderText("Password")
        self.password_input.setEchoMode(QLineEdit.Password)  # Mask password
        form_layout.addWidget(self.password_input)

        login_button = QPushButton("Login")
        login_button.clicked.connect(self.check_credentials)
        form_layout.addWidget(login_button)

        # Main layout
        main_layout = QVBoxLayout()
        main_layout.addWidget(form_widget)
        main_layout.setAlignment(Qt.AlignCenter)  # Center align the form widget
        self.setLayout(main_layout)

    def check_credentials(self):
        username = self.username_input.text()
        password = self.password_input.text()

        user = self.db_manager.get_user(username)
        if user and user[2] == password:
            self.accept()
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid credentials")

class SteriBotGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Database Initialization
        self.db_manager = DatabaseManager("SteriBot-DB.db")

        # Splash Screen
        splash = SplashScreen()
        if splash.exec_() != QDialog.Accepted:
            sys.exit()  # Quit if splash screen is closed

        # Login Screen
        login = LoginScreen(self.db_manager)
        if login.exec_() != QDialog.Accepted:
            sys.exit()  # Quit if login fails

        self.setWindowTitle("SteriBot Dashboard")
        self.setGeometry(0, 0, QApplication.desktop().screenGeometry().width(), QApplication.desktop().screenGeometry().height())  # Set window size to fullscreen

        # Maximize window
        self.showMaximized()

        # Central Widget and Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Tab Widget
        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        # Tab 1: Coordinates Control
        self.setup_coordinates_tab(tabs)

        # Tab 2: Automated Sterilization
        self.setup_sterilization_tab(tabs)

        # Tab 3: Robot Configurations
        self.setup_configurations_tab(tabs)

    def setup_coordinates_tab(self, tabs):
        tab = QWidget()
        tabs.addTab(tab, "Manual Control")
        layout = QGridLayout()
        tab.setLayout(layout)

        instruction_label = QLabel("Use arrow keys for movement control.\nPress 'Q' for clockwise rotation and 'E' for anti-clockwise rotation.")
        layout.addWidget(instruction_label, 0, 0, 1, 2)

    def setup_sterilization_tab(self, tabs):
        tab = QWidget()
        tabs.addTab(tab, "Sterilization")
        layout = QVBoxLayout()
        tab.setLayout(layout)

        room_label = QLabel("Enter Room Numbers (comma-separated):")
        layout.addWidget(room_label)

        self.room_input = QLineEdit()
        layout.addWidget(self.room_input)

        sterilize_button = QPushButton("Start Sterilization")
        sterilize_button.clicked.connect(self.start_sterilization)
        layout.addWidget(sterilize_button)

    def setup_configurations_tab(self, tabs):
        tab = QWidget()
        tabs.addTab(tab, "Robot Configurations")
        layout = QVBoxLayout()
        tab.setLayout(layout)

        wheel_label = QLabel("Wheel Velocities:")
        layout.addWidget(wheel_label)

        # Add controls for wheel velocities (sliders, input boxes, etc.)

        uv_label = QLabel("UV Light Intensity:")
        layout.addWidget(uv_label)

        # Add control for UV light intensity (slider, input box, etc.)

    def start_sterilization(self):
        room_numbers = self.room_input.text().split(',')
        # Start sterilization process for each room number
        for room_number in room_numbers:
            print(f"Sterilizing room: {room_number}")

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Up:
            print("Move forward")
        elif key == Qt.Key_Down:
            print("Move backward")
        elif key == Qt.Key_Left:
            print("Move left")
        elif key == Qt.Key_Right:
            print("Move right")
        elif key == Qt.Key_Q:
            print("Clockwise rotation")
        elif key == Qt.Key_E:
            print("Anti-clockwise rotation")

# Main execution
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SteriBotGUI()
    window.show()
    sys.exit(app.exec_())