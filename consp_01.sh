#sleep 3
#sudo python movement_standup2.py -c 1 -x 1
#sleep 5
#sudo python movement_walk.py -c 1 -d Front -st 3 -so 0
#sudo python movement_sitdown2.py -c 0 -x 1
#sudo python cmd_headaction.py -a 3
#sudo python movement_head2.py -c 1 -m 1 -ns -0.4 -ne -0.4 -nm 0
#sudo python movement_head2.py -c 1 -m 2 -ns -0.4 -ne -0.4 -nm 0
#sudo python movement_head2.py -c 1 -m 4 -ns -0.4 -ne -0.4 -nm 0
sleep 2
#sudo python movement_sitdown2eat.py -c 1 -x 1 -st 3
sudo python movement_walk.py -c 1 -d Front -st 10 -so 0
