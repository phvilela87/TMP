# TMP

Projeto da Graduação de Sistemas de Controle e Automação para Monitoramento de carga de caminhão

## Habilita/Desabilita a abertura da porta do baú através de um SW utilizando um broker MQTT

truckWeight: truck weight in kg
alarmStatus: 0 - Alarm OFF ; 1 - Alarm ON

### JSON template:

```json
{
    "truckWeight" : 565,
    "alarmStatus  : 0
}
```

## Recebe notificações sobre status de violação do sistema e peso atual do caminhão

doorCommand: 0 - unlock the door ; 1 - lock the door
truckWeight: truck weight in kg
alarmCtrl: 0 - does nothing ; 1 - deactivate the alarm
### JSON template:

```json
{
    "doorCommand" : 1,
    "truckWeight" : 565,
    "alarmCtrl    : 0
}
```

### Additional Information 

project name: Atmel Studio PRojects <br/>
version: v0r1 <br/> 
author: Pedro Vilela (pvilela)  
date: 15/10/2019
