import threading


class Thread1(threading.Thread):  
	def __init__(self, cond):  
		threading.Thread.__init__(self)
  
	def run(self):  
		runA()
		self.cond.notify()
class Thread2(threading.Thread):  
	def __init__(self, cond):  
		threading.Thread.__init__(self) 
  
	def run(self):  
		runB()
		self.cond.notify()

class Thread3(threading.Thread):  
	def __init__(self, cond):  
		threading.Thread.__init__(self)
  
	def run(self):
		runC()
     
def runA():
	while True:
		print ('Filtrar y medir corriente')

def runB():
	while True:
		wait_for()
		print ('Filtar y medir Voltaje')
		
def kelly():
	

def runC():
	wait_for(kelly)
	while True:
		print('Realizar PID')
		

if __name__ == "__main__":
	
	cond = threading.Condition() 
	t1 = Thread1(cond)
	t2 = Thread2(cond)
	t3 = Thread3(cond)
	
	t1.start()
	t2.start()    
	t3.start()
