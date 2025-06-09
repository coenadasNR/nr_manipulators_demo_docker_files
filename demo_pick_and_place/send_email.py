import smtplib
from email.message import EmailMessage

from string import Template
from pathlib import Path

email = EmailMessage()

email['from'] = 'NR Manipulators'
email['to'] = '****@gmail.com'
email['subject'] = 'Feedback Received'

email.set_content = "Hello"


with smtplib.SMTP(host='smtp.gmail.com', port=587) as smtp:
    smtp.ehlo()
    smtp.starttls()
    smtp.login('****@gmail.com','****')
    smtp.send_message(email)