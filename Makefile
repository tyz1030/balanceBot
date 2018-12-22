all:                                                         
	@make -C lcmtypes --no-print-directory
	@make -C balancebot --no-print-directory
	@make -C measure_motors --no-print-directory
	@make -C test_motors --no-print-directory
	@make -C optitrack/common --no-print-directory
	@make -C optitrack --no-print-directory
	@make -C optitrack_example --no-print-directory                                                 

opti:
	@make -C lcmtypes --no-print-directory
	@make -C optitrack/common --no-print-directory
	@make -C optitrack --no-print-directory
	@make -C optitrack_example --no-print-directory

lcmtypes:
	@make -C lcmtypes

lcmspy:	
	/bin/bash setenv.sh
	@make -C lcmtypes                                                          
	@make -C java 

clean:                                             
	@make -C balancebot -s clean
	@make -C measure_motors -s clean
	@make -C test_motors -s clean
	@make -C lcmtypes -s clean       
	@make -C optitrack -s clean
	@make -C optitrack/common -s clean 
	@make -C optitrack_example -s clean                                        
	@make -C java -s clean
