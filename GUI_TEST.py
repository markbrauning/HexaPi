import PySimpleGUI as sg
import time

x= 1
sg.theme('Dark')
#size=(20, 1), 
layout = [[sg.Text('Loop Counter:'), sg.Text(key='HEHE')],
          [sg.Text('Event type: '), sg.Text(key='line2')],
          [sg.Button('Exit')]]
          
# Create the Window
window = sg.Window('Window Title', layout)
# Event Loop to process "events"
while True:             
	x = x + 1
	#event, values = window.read(timeout=0)
	#print(event, values)
	if event == sg.WIN_CLOSED or event == 'Exit':
		break
	window['HEHE'].update(str(x))
	window['line2'].update(str(event))

window.close()
