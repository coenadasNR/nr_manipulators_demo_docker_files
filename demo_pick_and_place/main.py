from flask import Flask, render_template, request, redirect, jsonify
import subprocess
import os
from subprocess import Popen, PIPE
from email.message import EmailMessage
import smtplib

cwd = os.getcwd()

# cwd = "/home/nr-ws-1/Desktop/nr_manipulator_demo_docker_files/demo_pick_and_place"

# Specify the container name or ID
CONTAINER_NAME = "demo_pick_and_place"
# CONTAINER_NAME = "demo2"

def pick_place_sim():
    os.environ['RUN_MODE'] = "sim"
    print(cwd)
    subprocess.call(f"{cwd}/demo2.sh", shell=True)
def pick_place_sim_real():
    os.environ['RUN_MODE'] = "hardware"
    subprocess.call(f"{cwd}/demo2.sh", shell=True)
# def demo2_pick_place_sim_real():
#     os.environ['RUN_MODE'] = "hardware"
#     subprocess.call(f"{cwd}/run.sh", shell=True)

def send_email(data):
    if request.method == 'POST':
        data = request.form.to_dict()
        email = EmailMessage()
        email['from'] = 'NR Manipulators'
        email['to'] = '****@gmail.com' #TODO : Enter Email ID of person who wants to receive these
        email['subject'] = 'Feedback Received'
        email.set_content(f"Name : {data['name']}\nEmail : {data['email']}\nMessage : {data['message']}")
        with smtplib.SMTP(host='smtp.gmail.com', port=587) as smtp: # DO NOT CHANGE PORT NUMBER
            smtp.ehlo()
            smtp.starttls()
            smtp.login('****@gmail.com','****') #TODO : ENter email id and password of an account that will send these messages
            smtp.send_message(email)
        return "Email Sent!"
            

def write_to_file(data):
    with open('database.txt',mode='a') as database:
        name = data["name"]
        email= data["email"]
        message = data["message"]
        file = database.write(f'\n{name},{email},{message}')


app = Flask(__name__)

@app.route('/')
def run_demo():
    return render_template('index.html')
@app.route('/<string:page_name>')
def html_page(page_name):
    return render_template(page_name)
@app.route('/start_demo_sim',methods=['POST','GET'])
def sim():
    if request.method == 'POST':
        pick_place_sim()
        return redirect('/index.html')
    else:
        return 'something went wrong. Try again!'


@app.route('/start_demo_real',methods=['POST','GET'])
def real():
    if request.method == 'POST':
        pick_place_sim_real()
        return redirect('/index.html')
    else:
        return 'something went wrong. Try again!'

@app.route('/submit_form',methods=['POST','GET'])
def submit_form():
    if request.method == 'POST':
        data = request.form.to_dict()
        write_to_file(data)
        send_email(data)
        return redirect('/thankyou.html')
    else:
        return 'something went wrong. Try again!'
    
@app.route('/stop_container', methods=['POST'])
def stop_demo():
    subprocess.call(f"docker kill {CONTAINER_NAME}", shell=True)
    return redirect('/thankyou.html')


    

if __name__=="__main__":
    app.run(host='0.0.0.0',port=2003, debug=True)