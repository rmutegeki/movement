#!/usr/bin/env python
'''
Created on 2022. 1. 7.
@author: macroact

## data field discription
0. code
1. movement step
2 .movement discription
3. speed (0~20)
4. accel (0~5)
5. time sleep
6. 
7. 
8.
9. head movement code
10. front_left_pan (f/-1.57, s/0,  b/1.57)
11. front_left_shoulder (d/0.25, s/0, u/-0.7)
12. front_left_leg (d/-1.57, u/0.75) 
13. front_right_pan (f/1.57, s/0,  b/-1.57) 
14. front_right_shoulder (d/0.25, s/	0, u/-0.7)
15. front_right_leg (d/1.57, u/-0.75)
16. rear_left_pan (f/-1.57, s/0, b/1.57)
17. rear_left_shoulder (d/0.25, s/0, u/-0.7)
18. rear_left_bridge (s/0.8 ~ u/-0.6)
19. rear_left_leg (s/-1.57 ~ u/-0.3)
20. rear_right_pan (f/1.57, s/0, b/-1.57)
21. rear_right_shoulder (d/0.25, s/0, u/-0.7)
22. rear_right_bridge (s/-0.8 ~ u/0.6) 
23. rear_right_leg (s/1.57 ~ u/0.3)
24. neck_lift  (u/0.8 ~ d/-0.7)
25. neck_pan   (l/1.57, s/0, r/-1.57)
26. head_lift  (u/-1.35 ~ d/-0.65)
27. tail_lift  (d/1 ~ u/-0.5)

'''

class OptsetConfig:

    JOINT_GAP_0 = [
##      0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15	16	17
# test robot - 67
#        0,	0,	0.3,	-0.1,	-0.05,	-0.3,	0,	0,	0,	0,	-0.15,	-0,	-0.05,	0.15,	0,	0.05,	0,	0  
# black - 67 
        0,	0,	0.2,	0.1,	0.2,	-0.1,	-0.1,	0.1,	-0.25,	-0.25,	0,	-0.05,	0.2,	0.2,	0.5,	0,	0,	0  
# white1 - 139
#        0,	-0.05,	0.3,	-0.05,	0,	-0.3,	0.2,	-0.15,	0,	-0.35,	-0.3,	0.15,	-0.1,	0.15,	0.2,	0.1,	0,	0  
# white2 - 139
#        0.1,	-0,	0.2,	-0.2,	0.2,	-0.4,	0.2,	0.1,	-0.1,	-0.2,	-0.2,	-0.05,	0,	0.2,	-0.5,	-0.1,	-0.1,	0  

    ]
