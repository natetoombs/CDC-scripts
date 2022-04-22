import sys

# 1. Import `QApplication` and all the required widgets
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtWidgets import QPushButton, QLineEdit, QHBoxLayout

def stopFcn():
    msg.setText("Stopping")

# 2. Create an instance of QApplication
app = QApplication(sys.argv)

# 3. Create an instance of your application's GUI
window = QWidget()
window.setWindowTitle('PyQt5 App')
window.setGeometry(100, 100, 600, 80)
# window.move(300, 150)
# helloMsg = QLabel('<h1>Hello World!</h1>', parent=window)
# helloMsg.move(30, 15)

stopBtn = QPushButton('STOP')
stopBtn.clicked.connect(stopFcn)
msg = QLabel(' ')

layout = QHBoxLayout()
layout.addWidget(stopBtn)
layout.addWidget(msg)
window.setLayout(layout)

# 4. Show your application's GUI
window.show()

# 5. Run your application's event loop (or main loop)
sys.exit(app.exec_())