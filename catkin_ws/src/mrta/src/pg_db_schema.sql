\connect mrta;

CREATE TABLE IF NOT EXISTS results (
	_id                     SERIAL NOT NULL,
	robots                  INTEGER NOT NULL,
	tasks                   INTEGER NOT NULL,
    	pgraphs                 INTEGER NOT NULL,
	alpha                   NUMERIC(3,2) NOT NULL,
	beta			NUMERIC(3,2) NOT NULL,
	ms1			DOUBLE PRECISION,
	ms2         		DOUBLE PRECISION,
	tt1      		DOUBLE PRECISION,
	tt2     		DOUBLE PRECISION,  
	it1      		DOUBLE PRECISION,
	it2     		DOUBLE PRECISION,  
    scheduled_tasks1         	INTEGER,
    scheduled_tasks2        	INTEGER,
	last_updated		TIMESTAMP,
	comment			VARCHAR(20),				

	PRIMARY KEY (_id)
);
   
