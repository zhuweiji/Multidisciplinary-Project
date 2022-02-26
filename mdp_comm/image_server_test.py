import time
from mdp_comm.image import ImageServer

i = ImageServer(print)
i.run()

while not i.is_connected():
    time.sleep(1)

i.capture_and_send()
    
