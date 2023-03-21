# VERSIONI SOFTWARE  

## Versione 7.2   
codice spostiamo tasti fast/back wards su analogico per la nuova pcb    
Ffw: A4
Frw: A5
	
## Versione 7.3
codice aggiornato togliendo libreria stepper per non funzionamento    
Adozione metodo manuale con digitalwrite per tutti e 2 i motori.  
	  
	  
# Versione 7.4
Cambiato x lunghezza nastro da 40 a 6808    
Cambiata volumeperstep con valore per 1/32 microstep  
Cambio portata iniziale a 1 ml/h  
Insirto 2 us delay in fastforward, ma il ciok non cambia. Successivamente rimosso  
Camiato tempo iniziale 30 sec  
Cambiato DeltaT tempo 30 sec  
Scambiati DIR e STEP nel codice del motore web handling  
	  
# Versione 7.4.1  
Fix del problema della velocita' dei motori usando micros() invece che millis().  Da testare overflow  
  
# Versione 7.4.2      
Sostituita gestione del motore della pompa da micros() a intterrupt.   
	  
# Versione 7.5     
Modifiche al Log Data. Inseriti in output anche i parametri Voltaggio e portata. I parametri sono salvati continuamente, per capire quando stanno monitorando la filatura reale, si inserisce anche un flag (0: non sta filando, 1: sta filando). A questo punto   occorre solo trovare una strategia per 	correlare quale punto della carta del rullo collettore è esposta al processo in quel dato   momento.  
	  
# Versione 7.6   
Merge modifiche 7.4.2 e 7.5.  
Commentati comandi per servo.  
Corretto volumeperstep.  
Cambiata velocità di incremento della   portata da menù.    
Implementata la gestione di un DAC esterno MCP4725 per pilotare generatore Hv  
  
# Versione 7.7  
Implementata gestione esterna pwm per pilotare i servomotori con modulo pca9685  

# Versione 7.8  
aggiunto il parametro numero prova in logdata  
aggiunto relè al pin 6 per Vprog generatore Hv e relè ausiliario al pin 5  