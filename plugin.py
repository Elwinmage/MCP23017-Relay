# Manage relay USR-R16 or MCP23017 
# $Id: plugin.py 138 2020-07-29 08:58:48Z eric $
#
"""
<plugin key="Relay" name="Relay  plugin" author="morand" version="1.0.0" wikilink="" externallink="">
    <description>
        <h2>Relay</h2><br/>
          Manage your MCP23017 or USR-R16 relay<br/>
          You can specify a type of relay for each relay number. By default relay are simple ON/OFF.
          For example if Relay SELECTOR is "", Relay ON/OFF IMPULS is " 2" and Thermo  is "7,8" then Relay 1,3,4,5 and 6 are normal switches (ON/OFF), Relay number 2 is an impuls of 1s and relays 7 and 8 are Thermostats.<br />
        <h3>Parameters</h3><br />
        <ul>
          <li>Username: https login user name (USR-R16 only)</li>
          <li>Password: https password (USR-R16 only)</li>
          <li>Address: IP address for USR-R16 or I2C Address,MQTT IP for MCP23017</li>
          <li>Port : IP port (USR-R16 or MQTT if MCP23017 and Enabled)</li>
          <li>Type: Relay type</li>
          <li>Relay Number: Number of relays. If RelayNumber is set to 8 and relay type to MCP23017 then portA is used. If RelayNumber is negative and type set to MCP23017, then outputs are inverted</li>
          <li>Relay Selector: If in OFF MODE go in ON MODE, if in ON MODE go to OFF MODE (Normaly linked with a line status value)</li>
          <li>Relay Impuls: Send a 1s impulsion (ON -> Wait 1s -> OFF)</li>
          <li>Relay Thermo: Linked to a radiator it generates a thermostat. To work a temperature sensor must be declared in the same room. PID variables or Id of the temperature sensor can be manually changed in configuration file: domoticz/plugins/Relay/conf/pid_RelayId.json</li>
          <li>Debug: Enable or Disable Debug mode</li>
        </ul>
    </description>
    <params>
      <param field="Username" label="Username" width="150px"/>
      <param field="Password" label="Password" width="150px"/>
      <param field="Address" label="Address" width="150px"/>
      <param field="Port" label="Port" width="150px"/>
      <param field="Mode1" label="Type" width="90px">
        <options>
           <option label="MCP23017" value="MCP23017"/>
           <option label="USR-R16" value="USR-R16"  default="true" />
        </options>
      </param>
      <param field="Mode2" label="Relay Number"/>
      <param field="Mode3" label="SELECTOR relays" />
      <param field="Mode4" label="IMPULS relays" />
      <param field="Mode5" label="THERMO relays" />
      <param field="Mode6" label="Debug" width="150px">
            <options>
                <option label="True" value="Debug"/>
                <option label="False" value="Normal"  default="true" />
                <option label="Thermo debug" value="Thermo"/>
            </options>
        </param>
    </params>
</plugin>
"""

import Domoticz
import time
import binascii
import os
import sys
sys.path.append('/usr/lib/python3/dist-packages')
import smbus
import json
import sqlite3
import datetime
#import RPi.GPIO as GPIO
from mqtt import MqttClientSH2

# In Seconds 600 =>  10m
ThermoCycle=600

class Actuator:
    
    def __init__(self,addr):
        self._addr=addr
        
    def _relayState(self,devDomoticz,relayId,nValue,sValue):
        Domoticz.Debug(str(devDomoticz))
        #Switch type
        oldVal=devDomoticz.nValue
        devDomoticz.Refresh()
        newVal=devDomoticz.nValue
        if oldVal!=newVal:
            Domoticz.Log("!!!Relay  %d %s refresh from %d to %d"%(relayId,devDomoticz.Name,oldVal,newVal))            
        if devDomoticz.Type==244 and devDomoticz.SubType==72:
            if newVal!=nValue:
                self._send(relayId,'Switch')
                Domoticz.Debug("Switch %s"%devDomoticz.Name)
            else:
                Domoticz.Debug("!!!Relay  %d %s already in state %d/%s new state %d/%s %s %s "%(relayId,devDomoticz.Name,devDomoticz.nValue,devDomoticz.sValue,nValue,sValue,devDomoticz.LastLevel,devDomoticz.LastUpdate))
        #Impuls type
        elif devDomoticz.Type==244 and devDomoticz.SubType==5:
            #only send impuls if button is  a push button (SwitchType=9 or 10) or if new state is different of present state.
            if newVal!=nValue or devDomoticz.SwitchType in [9,10]:
                self._send(relayId,'On')
                self._send(relayId,'Off',delay=2)
                Domoticz.Debug("Impuls %s"%devDomoticz.Name)
        #ON/OFF Normal
        elif (devDomoticz.Type==244 and devDomoticz.SubType==73):
            self._send(relayId,sValue)
            Domoticz.Debug("ON/OFF => %s %s"%(sValue,devDomoticz.Name))
        else:
            Domoticz.Error("Type of Unit not supported: %d %d"%(devDomoticz.Type,devDomoticz.SubType))
            return
        devDomoticz.Update(nValue,sValue)

    def relayOn(self,devDomoticz,relayId):
        self._relayState(devDomoticz,relayId,1,'On')

    def relayOff(self,devDomoticz,relayId):
        self._relayState(devDomoticz,relayId,0,'Off')

    def updatePassword(self):
        pass

    def Reconnect(self):
        pass
    
class PID:
    def  ThermoLog(string):
        if Parameters["Mode6"] == "Thermo":
            Domoticz.Log(string)
        else:
            Domoticz.Debug(string)
            
    def __init__(self,switchId):
        self._boost=0
        self._lastMode=0
        self._heatTime=0
        self._error_sum=0.0
        self._prev_error=0.0
        self._switchId=switchId
        self._configFilename='%s/plugins/Relay/conf/pid_%d.json'%(os.getcwd(),switchId)
        if not os.path.isfile(self._configFilename):
            PID.ThermoLog('Creating %s'%(self._configFilename))
            with open(self._configFilename,'w+') as confFile:
                json.dump({"Kp":45,"Ki":20,"Kd":45,"Sensor":0},confFile,indent=4)
                confFile.close()
        with open(self._configFilename,'r') as confFile:
            self._pidConf=json.load(confFile)
            confFile.close()
        PID.ThermoLog('Init PID for %d: %s'%(switchId,str(self._pidConf)))
        #Associate Sensor and Actuator
        if self._pidConf['Sensor'] == 0:
            BasePlugin.dbCursor.execute('select DeviceStatus.ID from DeviceToPlansMap as one, DeviceToPlansMap as two, DeviceStatus where one.DeviceRowID=%d and one.PlanId=two.PlanID and DeviceStatus.ID=two.DeviceRowID and ((Type=80 and SubType=5) or (Type=82 and SubType=2))'%(self.getDomoticzId()+1))
            row=BasePlugin.dbCursor.fetchone()
            if row:
                self._pidConf['Sensor']=int(row[0])
                BasePlugin.dbCursor.execute('select sValue from DeviceStatus where ID=%d'%(self.getSensorId()))
                temperature=float(BasePlugin.dbCursor.fetchone()[0].split(';')[0])
                Domoticz.Log('Found a temperature of %.1f in the same room as device %d: %d'%(temperature,switchId,self.getSensorId()))
                self.__saveConf()
            else:
                Domoticz.Error('Can not find a temperature in the same room for device %d'%(switchId))

    def __saveConf(self):
        with open(self._configFilename,'w') as confFile:
            json.dump(self._pidConf,confFile,indent=4)
            confFile.close()

    def __getMode(self):
        return int(Devices[50+self._switchId].nValue)

    def saveLastMode(self):
        lm=self.__getMode()
        if (lm != 40):
            self._lastMode=lm
            
    def process(self):
        from time import time
        #Activate DB
        BasePlugin.dbConn.close()
        BasePlugin.dbConn=sqlite3.connect(os.getcwd()+'/domoticz.db')
        BasePlugin.dbCursor=BasePlugin.dbConn.cursor()
        #Check mode (Off=0, Night=10,Day=20,Auto=30,Boost=40)
        thermoMode=self.__getMode()
        orderType='day'
        if thermoMode == 0:
            self._boost=0
            return
        elif thermoMode == 10 :
            self._boost=0
            orderType='night'
        elif thermoMode==30:
            self._boost=0
            datet=datetime.datetime.now()
            time="%s:%s"%(str(datet.hour).rjust(2,'0'),str(datet.minute).rjust(2,'0'))
            query="SELECT Value FROM UserVariables WHERE Name LIKE 'thermo_"+str(Parameters["HardwareID"])+"_%_hour_"+str(self._switchId)+"' ORDER BY Name"
            BasePlugin.dbCursor.execute(query)
            rows = BasePlugin.dbCursor.fetchall()
            if rows:
                day=rows[0][0]
                night=rows[1][0]
                Domoticz.Debug("night: %s, day : %s"%(night,day))
                if (time > night or time < day):
                    orderType='night'
            else:
                Domoticz.Error("Can not find thermo_%s_*_hour_%d user variables"%(str(Parameters["HardwareID"]),self._switchId))
        elif thermoMode==40 and self._boost==0:
            Domoticz.Log("Boost mode for %s"%(Devices[50+self._switchId].Name))
            self._boost=4
        if self.getSensorId() ==  0:
            Domoticz.Error('No sensor linked to %d, so no process'%self._switchId)
            return
        #Get Temperature
        BasePlugin.dbCursor.execute('select Name,sValue from DeviceStatus where ID=%d'%(self.getSensorId()))
        row=BasePlugin.dbCursor.fetchone()
        if not row:
            Domoticz.Error("Can not get temperature for ID%d"%self.getSensorId())
            return
        mesure=float(row[1].split(';')[0])
        if orderType=='night':
            cmdOffset=150
        else:
            cmdOffset=100
        Devices[cmdOffset+self._switchId].Refresh()
        order=float(Devices[cmdOffset+self._switchId].sValue)
        
        PID.ThermoLog("PID %s order: %.1f (%s) mesure: %.1f"%(row[0],order,Devices[cmdOffset+self._switchId].Name,mesure))
        error=order-mesure
        self._error_sum+=error
        var_error=error-self._prev_error
        command=self._pidConf['Kp']*error+self._pidConf['Ki']*self._error_sum+self._pidConf['Kd']*var_error
        self._prev_error=error
        PID.ThermoLog("PID Command: %f"%command)
        #heatTime in seconds
        heatTime=command*4
        if heatTime>ThermoCycle :
            heatTime=ThermoCycle+10
            self._error_sum-=error
        elif self._boost>0:
            self._boost-=1
            if self._boost <=0:
                #go to last mode
                Devices[50+self._switchId].Update(nValue=self._lastMode,sValue=str(self._lastMode))
                #Reinit pid after boost
                self._error_sum=0
            else :
                heatTime=ThermoCycle+10                
        elif heatTime < 45:
            heatTime=0
            self._error_sum-=error
        self._heatTime=heatTime
        PID.ThermoLog("Heating %d during %ds/%d"%(self._switchId,heatTime,ThermoCycle))

    def heat(self):
        res = None
        Devices[self._switchId].Refresh()
        if self.__getMode()>0:
            if self._heatTime<0:
                self._heatTime=0
            else:
                Domoticz.Debug("Heating during %ds for %d/%d"%(self._heatTime,self._switchId,ThermoCycle))
            if self._heatTime > 0 and int(Devices[self._switchId].nValue)==0:
                res=[self._switchId,'On']
            elif self._heatTime == 0 and int(Devices[self._switchId].nValue)==1:
                res=[self._switchId,'Off']
            if self._heatTime > 0:
                self._heatTime-=10
        elif int(Devices[self._switchId].nValue)==1:
            res=[self._switchId,'Off']                
        return res
        
    def getSensorId(self):
        return self._pidConf['Sensor']

    def getDomoticzId(self):
        return int(Devices[self._switchId].ID)
    
class USR(Actuator):
    resendPwd=False
    
    def __init__(self,addr,port, user,passwd):
        Actuator.__init__(self,addr)
        self._port=port
        self._user=user
        self._passwd=passwd
        self._buffSize=16
        self._conn=Domoticz.Connection(Name="USR-R16",Transport="TCP/IP",Address=self._addr,Port=self._port,Protocol='None')
        self._conn.Connect()

    def Reconnect(self):
        self._conn.Connect()
        
    def _send(self,relayId,command,delay=0):
        if not self._conn.Connected():
            Domoticz.Error("Connection lost")
            if not self._conn.Connecting():
                self._conn.Connect()
            time.sleep(3)
            if not self._conn.Connected():
                Domoticz.Error("Connection is dead!")
                return 
        cmd='55aa000300'
        if command == 'On':
            cmd+='02'
        elif command == 'Off':
            cmd+='01'
        elif command == 'Switch' or command == 'Impuls':
            cmd+='03'
        else:
            Domoticz.Error("Unknowed command %s" %command)
        relay_id=''
        if len(hex(relayId))==3:
            relay_id='0'
        relay_id+=hex(relayId)[2:]
        cmd+=relay_id+'06'
        PID.ThermoLog("Socket Connection: %s:%s"%(self._addr,self._port))
        self._conn.Send(binascii.unhexlify(cmd),Delay=delay)

    def updatePassword(self):
        self._conn.Send(binascii.a2b_qp(self._passwd+'\r\n'))

        
class MCP23017(Actuator):
    
    IODIRA   = 0x00
    GPIOA    = 0x12
    OLATA    = 0x14
    
    def __init__(self,addr,portNumber):
        Actuator.__init__(self,addr)
        self.__invert=(portNumber<0)
        self._conn=smbus.SMBus(1)
        self.__initPort(0)
        if abs(portNumber) > 8:
            self.__initPort(1)

    def __initPort(self,port):
        dirConf = self._conn.read_byte_data(self._addr, MCP23017.IODIRA+port)        
        #All output
        if dirConf != 0x00:
            Domoticz.Error("Init port %d, old direction %s"%(port,bin(dirConf).rjust(8,'0')))
            if self.__invert:
                self._conn.write_byte_data(self._addr,MCP23017.OLATA+port,0xFF)
            self._conn.write_byte_data(self._addr,MCP23017.IODIRA+port,0x00)
            Domoticz.Debug("set init variable to 1")
            if not 150 in Devices:
                Domoticz.Device(Name="Relay Init",  Unit=150, TypeName="Switch",  Subtype=5, Switchtype=0, Image=9).Create()
            Devices[150].Update(sValue="On",nValue=1)
        
    def _send(self,relayId,command,delay=0):
        port   = int((relayId) / 9 )
        rId=relayId-8*port
        sMask=['0','0','0','0','0','0','0','0']
        sMask[(8-rId)]='1'
        mask   = int(''.join(sMask),2)
        Domoticz.Debug("Port: %d, Mask: %s"%(port,bin(mask)))
        status = self._conn.read_byte_data(self._addr, MCP23017.GPIOA+port)
        Domoticz.Debug("Current status for port %d is :%s" %(port,bin(status).rjust(8,'0')))
        newSatus=status
        if command=='On':
            if self.__invert:
                newStatus=status & ~mask
            else:
                newStatus=status|mask
        elif command=='Off':
            if self.__invert:
                newStatus=status|mask
            else:
                newStatus=status & ~mask
        elif command=='Switch' or command=='Impuls':
            newStatus=(status & ~mask) + (~(status & mask) & mask)
        else:
            Domoticz.Error("Command unknowed: %s" %(command))
        Domoticz.Debug("Sending to i2c:%s"%(bin(newStatus).rjust(8,'0')))
        if delay > 0:
            time.sleep(delay)
        #Check if MCP23017 must be re-initialized
        self.__initPort(port)
        self._conn.write_byte_data(self._addr,MCP23017.OLATA+port,newStatus)

class BasePlugin:
    enabled = False
    dbConn=None
    dbCursor=None
    
    def __init__(self):
        self._stopping=False
        self.actuator=None
        self._actuatorOnLine=True
        self.relayNb=16
        self.relaySwitch=[]
        self.relayImpuls=[]
        self.relayThermo=[]
        self.pids=[]
        self._nbPids=0
        self.addr=None
        self.port=None
        self.user=None
        self.passwd=None
        self._timer=0
        self._thermoStart=True
        self.mqttClient=None
        self._deleted=[]
        return

    def onStart(self):
        if Parameters["Mode6"] == "Debug":
            Domoticz.Debugging(1)
        Domoticz.Log("onStart called")
        DumpConfigToLog()

        #Activate DB
        BasePlugin.dbConn=sqlite3.connect(os.getcwd()+'/domoticz.db')
        BasePlugin.dbCursor=BasePlugin.dbConn.cursor()
        #
        if str(Parameters["Mode1"]) == 'MCP23017':
            Domoticz.Debug('Actuator type %s' %(Parameters["Mode1"]))
            self.actuator=MCP23017(int(Parameters['Address'].split(',')[0],16),int(Parameters["Mode2"]))
            #MQTT if enabled
            if len(Parameters["Address"].split(','))==2:
                import socket
                hostname=socket.gethostname().split('.')[0]
                self.base_topic = 'domoticz/slaves/'+hostname
                self.mqttserveraddress = Parameters["Address"].strip().split(',')[1]
                self.mqttserverport = Parameters["Port"].strip()
                Domoticz.Status("MQTT Domoticz Remote enabled on:"+self.base_topic+", "+self.mqttserveraddress+":"+self.mqttserverport)
                self.mqttClient = MqttClientSH2(self.mqttserveraddress, self.mqttserverport, "", self.onMQTTConnected, self.onMQTTDisconnected, self.onMQTTPublish, self.onMQTTSubscribed)
        elif str(Parameters["Mode1"]) == 'USR-R16':
            self.actuator=USR(Parameters['Address'],Parameters['Port'],Parameters['Username'],Parameters['Password'])
            self.passwd=Parameters['Password']
        else:
            Domoticz.Error('Actuator type %s unknowned' %(Parameters["Mode1"]))

        self.relayNb=abs(int(Parameters["Mode2"]))
        if Parameters["Mode3"]:
            self.relaySwitch=list(map(int,str(Parameters["Mode3"]).split(',')))
        if Parameters["Mode4"]:
            self.relayImpuls=list(map(int,str(Parameters["Mode4"]).split(',')))
        if Parameters["Mode5"]:
            self.relayThermo=list(map(int,str(Parameters["Mode5"]).split(',')))
        #CreateNew
        if (len(Devices)==0):
            self.__createDevices(range(1,self.relayNb+1))
        #Update
        else:
            to_update=[]
            for i in range(1,self.relayNb+1):
                if not i in  Devices:
                     to_update+=[i]
                elif (i in self.relaySwitch and not (Devices[i].Type==244 and Devices[i].SubType==72)) or \
                     (i in self.relayImpuls and not (Devices[i].Type==244 and Devices[i].SubType==5)) or \
                     (i in self.relayThermo and not (50+i) in Devices) or \
                     (i not in self.relayThermo and (50+i) in Devices) or \
                     ( (i not in self.relayImpuls and i not in self.relaySwitch) and not (Devices[i].Type==244 and Devices[i].SubType==73)):

                   Domoticz.Log("Status of Unit %d has changed" %i)
                   to_update+=[i]
                   #Delete Unit
                   if (50+i) in Devices:
                       #Delete Thermo
                       if self.mqttClient is not None:
                           self._deleted+=[Devices[50+i].ID]
                           self._deleted+=[Devices[100+i].ID]
                           self._deleted+=[Devices[150+i].ID]
                       Devices[(50+i)].Delete()
                       Devices[(100+i)].Delete()
                       Devices[(150+i)].Delete()
                       query='DELETE FROM UserVariables WHERE NAME LIKE "thermo_'+str(Parameters["HardwareID"])+'_%_hour_'+str(i)+'"'
                       BasePlugin.dbCursor.execute(query)
                       BasePlugin.dbConn.commit()
                       os.remove('%s/plugins/Relay/conf/pid_%d.json'%(os.getcwd(),i))
                   if self.mqttClient is not None:
                       self._deleted+=[Devices[i].ID]
                   Devices[i].Delete()
            if len(to_update)>0:
                Domoticz.Log(str(to_update))
                self.__createDevices(to_update)
        for i in self.relayThermo:
            self.pids+=[PID(i)]
        self._nbPids=len(self.pids)
            
    def __createDevices(self,relayIds):
          for i in relayIds:
                if i in self.relaySwitch:
                    Domoticz.Debug("Switch %d" %i)
                    Domoticz.Device(Name="Relay %d"%i,  Unit=i, TypeName="Switch",  Subtype=72, Switchtype=0, Image=9).Create()
                elif i in self.relayImpuls:
                    Domoticz.Debug("Impuls %d" %i)
                    Domoticz.Device(Name="Relay %d"%i,  Unit=i, TypeName="Switch",  Subtype=5, Switchtype=9, Image=9).Create()
                elif i in self.relayThermo:
                    Domoticz.Debug("ThermoSwitch %d" %i)
                    Domoticz.Device(Name="Thermo Relay %d"%i,  Unit=i, TypeName="Switch",  Switchtype=0, Image=9).Create()
                    Domoticz.Debug("Thermo %d" %(50+i))
                    Options = {"LevelActions": "||||",
                               "LevelNames": "Off|Nuit|Jour|Auto|Boost",
                               "LevelOffHidden": "false",
                               "SelectorStyle": "0"}
                    Domoticz.Device(Name="Relay Selector %d"%i, Unit=(50+i),TypeName="Selector Switch", Image=15,Options=Options).Create()
                    Domoticz.Device(Name="Relay Command %d - Confort"%i, Unit=(100+i),Type=242,Subtype=1).Create()
                    Domoticz.Device(Name="Relay Command %d - Eco"%i, Unit=(150+i),Type=242,Subtype=1).Create()
                    Devices[(100+i)].Update(nValue=20,sValue="20.0")
                    Devices[(150+i)].Update(nValue=17,sValue="17.0")
                    BasePlugin.dbCursor.execute('INSERT INTO UserVariables (name, ValueType, Value) VALUES ("thermo_%s_night_hour_%d",4,"21:00")'%(Parameters["HardwareID"],i))
                    BasePlugin.dbCursor.execute('INSERT INTO UserVariables (name, ValueType, Value) VALUES ("thermo_%s_day_hour_%d",4,"07:00")'%(Parameters["HardwareID"],i))
                    BasePlugin.dbConn.commit()
                else:
                    Domoticz.Debug("Normal %d" %i)
                    Domoticz.Device(Name="Relay %d"%i,  Unit=i, TypeName="Switch",  Switchtype=0, Image=9).Create()

                
    def onStop(self):
        Domoticz.Log("onStop called")
        self._stopping=True
        if BasePlugin.dbConn:
            BasePlugin.dbConn.close()

    def onConnect(self, Connection, Status, Description):
        Domoticz.Log("onConnect called")
        #MQTT ENABLED
        if self.mqttClient is not None:
            self.mqttClient.onConnect(Connection, Status, Description)
        else:
            if Status!=0:
                Domoticz.Error(Description)
                self._actuatorOnLine=False
            else:
                Domoticz.Debug('Connected')
                self._actuatorOnLine=True
                Connection.Send(binascii.a2b_qp(self.passwd+'\r\n'))

    def onMessage(self, Connection, Data):
        Domoticz.Debug("onMessage called")
        if self.mqttClient is not None:
            self.mqttClient.onMessage(Connection, Data)
        else:
            Domoticz.Log("Receiving data: %s"%(str(Data)))
            
    def onCommand(self, Unit, Command, Level, Hue):
        Domoticz.Debug("onCommand called for Unit " + str(Unit) + ": Parameter '" + str(Command) + "', Level: " + str(Level))
        Command = Command.strip()
        action, sep, params = Command.partition(' ')
        action = action.capitalize()
        if Unit==150:
            Domoticz.Debug("Set new status for Init: %s"%(action))
            if action=="On":
                Devices[Unit].Update(sValue="On",nValue=1)
            else:
                Devices[Unit].Update(sValue="Off",nValue=0)
        elif Unit > 50 :
            #self._thermoStart=True
            #If boost : by pass load balancing
            if Unit <100:
                sId=Unit-50
            elif Unit < 150:
                sId=Unit-100
            else :
                sId=Unit-150
            for pid in self.pids:
                if pid._switchId==sId:
                    pid.saveLastMode()
                    Devices[Unit].Update(int(Level),str(Level))
                    pid.process()
        elif (action == 'On'):
            self.actuator.relayOn(Devices[Unit],Unit)
        elif (action == 'Off'):
            self.actuator.relayOff(Devices[Unit],Unit)
        self.mqttPublish(Devices[Unit])
            
    def onNotification(self, Name, Subject, Text, Status, Priority, Sound, ImageFile):
        Domoticz.Log("Notification: " + Name + "," + Subject + "," + Text + "," + Status + "," + str(Priority) + "," + Sound + "," + ImageFile)

    def onDisconnect(self, Connection):
        Domoticz.Log("onDisconnect called")
        if self.mqttClient is not None:
            self.mqttClient.onDisconnect(Connection)
        elif self._stopping == False:
            self.actuatorOnLine=False

    def onMQTTConnected(self):
      if self.mqttClient is not None:
          self.mqttClient.subscribe([self.base_topic + '/in/#','domoticz/slaves'])

    def onMQTTDisconnected(self):
      Domoticz.Debug("onMQTTDisconnected")
      
    def onMQTTSubscribed(self):
        Domoticz.Debug("onMQTTSubscribed")

    def onMQTTPublish(self, topic, message): # process incoming MQTT statuses
        r_msg=json.loads(json.dumps(message))
        if(topic=="domoticz/slaves"):
            if (r_msg['cmd']=="update_all"):
                for dev in self._deleted:
                    msg=json.dumps({'idx':dev,"status":"Removed"})
                    self.mqttClient.publish(self.base_topic+'/out/req_info',str(msg),0)
                self._deleted=[]
                for dev in Devices:
                    self.mqttPublish(Devices[dev])
        elif topic==self.base_topic+'/in/req_info':
            for unit in Devices:
                if Devices[unit].ID==r_msg['idx']:
                    device=Devices[unit]
                    msg=json.dumps({"idx":device.ID,"status":"Ok","svalue":device.sValue,"nvalue":device.nValue,"name":device.Name,"type":device.Type,"subtype":device.SubType,"switchtype":device.SwitchType,"description":device.Description,'image':device.Image,'action':'write','options':device.Options})
                    self.mqttClient.publish(self.base_topic+'/out/req_info',str(msg),0)
                    return
            msg=json.dumps({"idx":r_msg['idx'],"status":"Not Found"})
            self.mqttClient.publish(self.base_topic+'/out/req_info',str(msg),0)
        elif topic==self.base_topic+'/in':
            for unit in Devices:
                if Devices[unit].ID==r_msg['idx']:
                    self.onCommand(unit,r_msg['command'],r_msg['level'],r_msg['color'])
                    

    def mqttPublish(self,device):
        if self.mqttClient is not None:
            msg=json.dumps({"idx":device.ID,"svalue":device.sValue,"nvalue":device.nValue})
            self.mqttClient.publish(self.base_topic+'/out',str(msg),0)
            
    def onHeartbeat(self):
        Domoticz.Debug("onHeartbeat called")
        if self.mqttClient is not None:
            if (self.mqttClient._connection is None) or (not self.mqttClient.isConnected):
                Domoticz.Debug("Reconnecting")
                self.mqttClient._open()
            else:
                self.mqttClient.ping()
        elif not self._actuatorOnLine:
            self.actuator.Reconnect()
        self._timer+=1
        if self._timer>=720:
            self.actuator.updatePassword()
            self._timer=0
        #PID management
        if self._nbPids>0 and ((self._timer%((ThermoCycle/10)/self._nbPids))==0 or self._thermoStart==True):
            self._thermoStart=False
            #Load Balancing
            pid=self.pids[int(self._timer%(ThermoCycle/10)/((ThermoCycle/10)/self._nbPids))]
            pid.process()
        for pid in self.pids:
            cmd=pid.heat()
            if cmd:
                Domoticz.Debug("%d %s"%(cmd[0],cmd[1]))
                if cmd[1]=='On':
                    self.actuator.relayOn(Devices[cmd[0]],cmd[0])
                else:
                    self.actuator.relayOff(Devices[cmd[0]],cmd[0])

    def onDeviceModified(self,Unit):
        Domoticz.Log("Device modified %d"%(int(Unit)))


global _plugin
_plugin = BasePlugin()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()

def onConnect(Connection, Status, Description):
    global _plugin
    _plugin.onConnect(Connection, Status, Description)

def onMessage(Connection, Data):
    global _plugin
    _plugin.onMessage(Connection, Data)

def onCommand(Unit, Command, Level, Hue):
    global _plugin
    _plugin.onCommand(Unit, Command, Level, Hue)

def onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile):
    global _plugin
    _plugin.onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile)

def onDisconnect(Connection):
    global _plugin
    _plugin.onDisconnect(Connection)

def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()

def  onDeviceAdded():
    global _plugin
    _plugin.onDeviceAdded()

def onDeviceModified(Unit):
    global _plugin
    _plugin.onDeviceModified(Unit)
    # Generic helper functions

def DumpConfigToLog():
    for x in Parameters:
        if Parameters[x] != "":
            Domoticz.Debug( "'" + x + "':'" + str(Parameters[x]) + "'")
    Domoticz.Debug("Device count: " + str(len(Devices)))
    for x in Devices:
        Domoticz.Debug("Device:           " + str(x) + " - " + str(Devices[x]))
        Domoticz.Debug("Device ID:       '" + str(Devices[x].ID) + "'")
        Domoticz.Debug("Device Name:     '" + Devices[x].Name + "'")
        Domoticz.Debug("Device nValue:    " + str(Devices[x].nValue))
        Domoticz.Debug("Device sValue:   '" + Devices[x].sValue + "'")
        Domoticz.Debug("Device LastLevel: " + str(Devices[x].LastLevel))
    return
