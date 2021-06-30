#!/usr/bin/env python3
# ^^^ Important - tells kattis this is python3 vs python2

# Test cmd : D:\Python\Planetoids\IcpcContestantAll\IcpcContestantAll\ICPC

###########
# IMPORTS #
###########
from time import perf_counter as clock
import sys
import json
import os.path
from time import sleep
import math as m

import traceback

def main():
    """
    layout
    
       C4       C3       C4         C3
    
             -------------------
             |  C1    |   C2   |
       C2    |        |        |    C1
             |-----------------|
             |  C3    |   C4   | 
       C4    |        |        |    C3
             -------------------
    
       C2       C1         C2       C1
       
    """
    
    
    ############
    # GLOBVARS #
    ############
    global SMALL
    global MEDIUM
    global LARGE
    
    global THRUST
    global CLOCKWISE
    global COUNTERCLOCKWISE
    global SHOOT
    global HYPERSPACE
    global UPDATE
    
    global MAX_X
    global MAX_Y

    global MIN_X
    global MIN_Y
    
    global MAP_HEIGHT
    global MAP_WIDTH
    
    global c
    
    
    
    
    global NB_FRAMES
    
    global default_only
    global thrust_only
    global turn_only
    global oturn_only
    global thrust_turn
    global thrust_oturn
    
    global SHIP_POS
    global ARTF_POS
    global SHIP_HDG
    
    global LAST_POS
    global SHIP_TRAJ
    
    global OBJECTIVE_PENDING
    global OBJECTIVE_POS
    global INIT_OBJECTIVE_POS
    
    global DETECT_WARPING
    global UFO_IN_SIGHT
    global UFO_POS
    
    
    
    global log_path
    global log_tr_path
    global log_err
    global log_tr
    
    
    SMALL = 48
    MEDIUM = 49
    LARGE = 50
    
    THRUST = 0
    CLOCKWISE = 0
    COUNTERCLOCKWISE = 0
    SHOOT = 1
    HYPERSPACE = 0
    UPDATE = 1
    
    MAX_X = 3800
    MIN_X = -3800
    
    MAX_Y = 2100
    MIN_Y = -2100
    
    MAP_HEIGHT = MAX_Y - MIN_Y
    MAP_WIDTH = MAX_X - MIN_X
    
    
    c = [0 for _ in range(5)] 
    c[1] = {"lower_left" : (MIN_X,0) , "upper_left" : (MIN_X,MAX_Y), "upper_right" : (0,MAX_Y), "lower_right" : (0,0)}
    c[2] = {"lower_left" : (0,0) , "upper_left" : (0,MAX_Y), "upper_right" : (MAX_X,MAX_Y), "lower_right" : (MAX_X,0)}
    c[3] = {"lower_left" : (MIN_X,MIN_Y) , "upper_left" : (MIN_X,0), "upper_right" : (0,0), "lower_right" : (0,MIN_Y)}
    c[4] = {"lower_left" : (0,MIN_Y) , "upper_left" : (0,0), "upper_right" : (MAX_X,0), "lower_right" : (MAX_X,MIN_Y)}
    
    
    
    
    ##################
    # LOG MANAGEMENT #
    ##################


    log_path = "D:\Python\Planetoids\IcpcContestantAll\IcpcContestantAll\ICPC\Logs\log_"
    
    i = 1
    # while os.path.isfile(log_path+str(i)+".txt"):
    #     i += 1
    log_tr_path = log_path + str(i) + "_trace" + ".txt"
    err = log_path + "err.txt"
    log_path += str(i) + ".txt"
    
    log_tr = open(log_tr_path,"w").close()
    log_tr = open(log_tr_path,"a")
    
    
    
    
    
    ##############
    # MISC FUNCS #
    ###############
    
    def output_gen(vals): #generate input when vals is input values in correct order
        return "".join( list(map(str,vals)) )
     
    def write(texte,fin="\n"): #write into log_trace.txt \n already in
        log_tr.write(texte + fin)
    
    def which_zone(pos): #outputs the zone (1,2,3 or 4) in which the point of coordinates pos=(x,y) is situated
        x,y = pos
        if x > 0 :
            if y > 0:
                return 2 #zone 2
            return 4 #zone 4
        if y > 0:
            return 1 #zone 1
        return 3 #zone 3

    def polar_to_cartesian(r,theta): #r,theta to x,y
        theta += 180
        theta%360
        theta = m.radians(theta) - m.pi    
        x = r * m.cos(theta)
        y = r * m.sin(theta)
        return x,y
    
    def cartesian_to_polar(pos1,pos2): #polar coordinates to join pos2 from pos1
        a,b = pos1
        c,d = pos2
        x = c-a
        y = d-b
        den = x + m.sqrt(x**2 + y**2)
        if den == 0.0:
            y += 0.01
            den = x + m.sqrt(x**2 + y**2)
        
        theta = y / den
        theta = m.atan(theta)
        theta *= 2
        
        theta = m.degrees(theta)
    
        if theta < 0 :
            theta = 360 + theta
        
        r = m.sqrt(x**2 + y**2)
        return r,theta
    
    def extend(coord): #outputs extended coordinates of an object (including the original ones)
        x,y = coord
        # if not ((x > MAX_X / 2 or x <MIN_X / 2) and (y > MAX_Y / 2 or y <MIN_Y / 2)):
        #     return [coord]
        zone = which_zone(coord)
        
        xLL = abs( c[zone]["lower_left"][0] - x )
        yLL = abs( c[zone]["lower_left"][1] - y )
        
        xLR = abs( c[zone]["lower_right"][0] - x )
        yLR = abs( c[zone]["lower_right"][1] - y )
        
        xUL = abs( c[zone]["upper_left"][0] - x )
        yUL = abs( c[zone]["upper_left"][1] - y )
        
        xUR = abs( c[zone]["upper_right"][0] - x )
        yUR = abs( c[zone]["upper_right"][1] - y )
        
        nouvCoord = [coord]
        
        if zone == 1: 
            zoneInit = 2
            refZoneInit = c[zoneInit]["lower_right"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xLL, yLL
            xP = xRef + xP
            yP = y
            nouvCoord.append( (xP,yP) )
            
            zoneInit = 3
            refZoneInit = c[zoneInit]["lower_right"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xUR, yUR
            xP = x
            yP = yRef - yP
            nouvCoord.append( (xP,yP) )
            
            zoneInit = 4
            refZoneInit = c[zoneInit]["lower_right"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xUL, yUL
            xP = xRef + xP
            yP = yRef - yP
            nouvCoord.append( (xP,yP) )
            
        elif zone == 2: 
            zoneInit = 1
            refZoneInit = c[zoneInit]["lower_left"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xLR, yLR
            xP = xRef - xP
            yP = yP
            nouvCoord.append( (xP,yP) )
            
            zoneInit = 3
            refZoneInit = c[zoneInit]["lower_left"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xUR, yUR
            xP = xRef - xP
            yP = yRef - yP
            nouvCoord.append( (xP,yP) )
            
            zoneInit = 4
            refZoneInit = c[zoneInit]["lower_left"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xUL, yUL
            xP = x
            yP = yRef - yP
            nouvCoord.append( (xP,yP) )
            
        elif zone == 3: 
            zoneInit = 1
            refZoneInit = c[zoneInit]["upper_right"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xLR, yLR
            xP = x
            yP = yRef + yP
            nouvCoord.append( (xP,yP) )
        
            zoneInit = 2
            refZoneInit = c[zoneInit]["upper_right"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xLL, yLL
            xP = xRef + xP
            yP = yRef + yP
            nouvCoord.append( (xP,yP) )
        
            zoneInit = 4
            refZoneInit = c[zoneInit]["lower_right"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xLL, yLL
            xP = xRef + xP
            yP = y
            nouvCoord.append( (xP,yP) )
        
        else: #zone = 4
            zoneInit = 1
            refZoneInit = c[zoneInit]["upper_left"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xLR, yLR
            xP = xRef - xP
            yP = yRef + yP
            nouvCoord.append( (xP,yP) )
            
            zoneInit = 2
            refZoneInit = c[zoneInit]["upper_left"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xLL, yLL
            xP = x
            yP = yRef + yP 
            nouvCoord.append( (xP,yP) )
            
            zoneInit = 3
            refZoneInit = c[zoneInit]["upper_left"]
            xRef, yRef = refZoneInit[0], refZoneInit[1]
            xP, yP = xUR, yUR
            xP = xRef - xP
            yP = y
            nouvCoord.append( (xP,yP) )
        
        return nouvCoord
    
    def distance(c1,c2):
        a,b = c1
        c,d = c2
        
        return m.sqrt( (c-a)**2 + (d-b)**2 )
    
    def traj_angle(): #takes global lastPos
        global LAST_POS,SHIP_POS
        _,angle = cartesian_to_polar( LAST_POS,SHIP_POS )
        LAST_POS = SHIP_POS
        if LAST_POS == (-1,0):
            return 90
        return angle    
    
    def angle_diff_signed(heading,aim): #Returns minus angle if direction needed is counterclockwise, positive angle if direction clockwise 
        angle = (aim - heading + 540) % 360 - 180
        return -angle
        
    def calcul_angle_limite(dist,precision):
        return (dist+precision) * 0.09 - 40

    def choix_pos(tab_pos,shipPos,shipHdg,shipTraj): #On selectionne l'element correct le plus proche, sinon celui avec l'angle le plus petit
        bestPos = -1
        diffTraj = angle_diff_signed(shipTraj,shipHdg)
        if abs(diffTraj) > 45:
            ref = shipTraj
        else:
            ref = shipHdg
    
    
        distLimit = 2000
    
    
        vals = []
        for i,aimPos in enumerate(tab_pos):
            distA,aimAngle = cartesian_to_polar(shipPos,aimPos)
            diffA = angle_diff_signed(ref,aimAngle)
            stack = (distA, abs(diffA),i)
            vals.append(stack)
    
        vals.sort()
        
        
        
        
        #New calcul
        reste = []
        for el in vals:
            dist,agl,i = el
            if dist > distLimit:
                bestPos = i
                break
            
            else:
                angleLimite = calcul_angle_limite(dist,0)
                if agl < angleLimite:
                    bestPos = i
                    break
            reste.append( (agl,i) )
        
        reste.sort()
        if bestPos == -1:
            bestPos = reste[0][1]
    
        
    
        #Old calcul
        bestPos = vals[0][2]
        
        return tab_pos[bestPos]





    def normalize(angle):
        angle %= 360
        if angle < 0 :
            angle = 360 - angle
        return angle
    





    #############
    # BEHAVIORS # A behaviors takes data as arguments, outputs an input string for the ship
    #############
    
    
    def seek_artf():
        global OBJECTIVE_POS
        global SHIP_POS
        global ARTF_POS
        global LAST_ARTF_POS
        global SHIP_HDG
        global SHIP_TRAJ
    
        r, aim_traj = cartesian_to_polar(SHIP_POS,OBJECTIVE_POS)
        
        if r < 500:
            precision = 10
        elif 500 <=r <2000:
            precision = 20
        else:
            precision = 25
        max_drift = 110 
        
        
        #NEW
        if r > 4000:
            max_drift = 30
        elif 4000 <= r <= 3000:
            max_drift = 50
        elif 3000 < r <= 2000:
            max_drift = 75
        elif 2000 < r <= 1000:
            max_drift = 95
        else:
            max_drift = 120
        
        
        max_drift_exceed = False
        diffTraj = angle_diff_signed(SHIP_TRAJ,aim_traj) #Entre traj actuelle et traj pour rejoindre l'objectif
        
        diffDirection = angle_diff_signed(SHIP_HDG, SHIP_TRAJ)
        if abs(diffDirection) > max_drift:
            max_drift_exceed = True
    
        
        if abs(diffTraj) > precision:
            if diffTraj > 0:
                out = thrust_turn

            else:
                out = thrust_oturn
            
        else:
            if abs(diffDirection) > precision:
                if diffDirection > 0:
                    out = thrust_turn
                else:
                    out = thrust_oturn
            else:
                out = thrust_only
                
                
        if max_drift_exceed:
            out = thrust_only
            
        if SHIP_POS == [0,0] and abs(angle_diff_signed(SHIP_HDG, aim_traj)) > precision:
            write("inLoop")
            if 90 < aim_traj < 270:
                out = oturn_only
            else:
                out = turn_only
            
                    
        return out
            
    def seek_ufo():
        global OBJECTIVE_POS
        global SHIP_POS
        global UFO_POS
        global LAST_ARTF_POS
        global SHIP_HDG
        global SHIP_TRAJ
        OBJECTIVE_POS = UFO_POS
    
        r, aim_traj = cartesian_to_polar(SHIP_POS,OBJECTIVE_POS)
        
        if r < 500:
            precision = 10
        elif 500 <=r <2000:
            precision = 20
        else:
            precision = 25
        max_drift = 110 
        
        
        #NEW
        if r > 4000:
            max_drift = 30
        elif 4000 <= r <= 3000:
            max_drift = 50
        elif 3000 < r <= 2000:
            max_drift = 75
        elif 2000 < r <= 1000:
            max_drift = 95
        else:
            max_drift = 120
        
        
        max_drift_exceed = False
        diffTraj = angle_diff_signed(SHIP_TRAJ,aim_traj) #Entre traj actuelle et traj pour rejoindre l'objectif
        
        diffDirection = angle_diff_signed(SHIP_HDG, SHIP_TRAJ)
        if abs(diffDirection) > max_drift:
            max_drift_exceed = True
    
        
        if abs(diffTraj) > precision:
            if diffTraj > 0:
                out = thrust_turn

            else:
                out = thrust_oturn
            
        else:
            if abs(diffDirection) > precision:
                if diffDirection > 0:
                    out = thrust_turn
                else:
                    out = thrust_oturn
            else:
                out = thrust_only
                
                
        if max_drift_exceed:
            out = thrust_only
            
        if SHIP_POS == [0,0] and abs(angle_diff_signed(SHIP_HDG, aim_traj)) > precision:
            write("inLoop")
            if 90 < aim_traj < 270:
                out = oturn_only
            else:
                out = turn_only
            
                    
        return out
    
    
    
    def kill_ufo(pos,aim,hdg,dist):
        global OBJECTIVE_PENDING
        lim_tir = 5
        rang = 2400
        if dist > rang:
            return seek_ufo(pos)
        
        angle = angle_diff_signed(hdg,aim)
        if abs(angle) > lim_tir:
            if angle > 0:
                return turn_only
            return oturn_only
        
        OBJECTIVE_PENDING = False
        return thrust_only
    
    ########
    # BODY #
    ########

    
    NB_FRAMES = 1
    nothing = "000001"
    default_out = "000101"
    thrust_only = "100101"
    turn_only =   "010101"
    oturn_only =   "001101"
    thrust_turn = "110101"
    thrust_oturn ="101101"
    

    default_out_ns = "000001"
    thrust_only_ns = "100001"
    turn_only_ns =   "010001"
    oturn_only_ns =   "001001"
    thrust_turn_ns = "110001"
    thrust_oturn_ns ="101001"
    
    SHIP_POS = (-1,-1)
    ARTF_POS = (-1,-1)
    SHIP_HDG = 90
    
    LAST_POS = (-1,0) #to get initial 90
    SHIP_TRAJ = 90
    
    OBJECTIVE_PENDING = False
    OBJECTIVE_POS = (MIN_X-1,MIN_Y-1)
    INIT_OBJECTIVE_POS = (MIN_X-1,MIN_Y-1)
    

    DETECT_WARPING = False
    
    UFO_IN_SIGHT = False
    
    while True:
        raw_data = sys.stdin.readline()
        # Exit if stdin is closed.
        if not raw_data:
            break
        data = json.loads(raw_data)
        # Exit if we hit Game Over.
        if "gameOver" in data and data["gameOver"]:
            continue
    
        #UPDATE GLOBAL CONSTANTS
        SHIP_POS = data["shipPos"]
        ARTF_POS = data["artfPos"]
        SHIP_HDG = data["shipR"]
        
        # DETECT_WARPING = False
        # if abs(SHIP_POS[0] - LAST_POS[0]) > MAX_X or abs(SHIP_POS[1] - LAST_POS[1]) > MAX_Y:
        #     DETECT_WARPING = True
        
        
        SHIP_TRAJ = traj_angle()
    
    
    
        ##############################
        # @TODO: Process input frame #
        ##############################
        if ARTF_POS != OBJECTIVE_POS and not UFO_IN_SIGHT:
            OBJECTIVE_PENDING = False
        
        if not OBJECTIVE_PENDING:
            tab = extend(ARTF_POS)
            INIT_OBJECTIVE_POS = choix_pos(tab,SHIP_POS,SHIP_HDG,SHIP_TRAJ)
            OBJECTIVE_POS = INIT_OBJECTIVE_POS
            OBJECTIVE_PENDING = True
        
        if DETECT_WARPING:
            OBJECTIVE_POS = ARTF_POS
    

        ufos = data['ufoPos']
        ufoSizes = data["ufoSizes"]
        ufoPos = []
        for i,pos in enumerate(ufos):
            if ufoSizes[i] > 48:
                ufoPos.append(pos)
        
        if ufoPos != []:
            ufoPos = ufoPos[0]
        
        dist,pos_angle = cartesian_to_polar(SHIP_POS, ufoPos) 
        aim_angle = angle_diff_signed(SHIP_HDG,pos_angle)
        UFO_IN_SIGHT = False
        if abs(aim_angle) < 45:
            UFO_IN_SIGHT = True
            
        if not UFO_IN_SIGHT:
            out = seek_artf()
        else:
            UFO_POS = ufoPos
            OBJECTIVE_POS = ufoPos
            out = kill_ufo(UFO_POS,aim_angle,SHIP_HDG,dist)
    

        #################
        # Emit command. #
        #################
    
        sys.stdout.write(out + "\n")
    
    
    
        sys.stdout.flush()
        log_tr.flush()
        
        NB_FRAMES += 1
    
    
        
        
try:
    main()
except Exception :
    traceback.print_exc(file = log_tr)

