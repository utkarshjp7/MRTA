\connect mrta;

DROP TABLE results;

CREATE TABLE IF NOT EXISTS results (
	_id                         SERIAL NOT NULL,
	robots                      INTEGER NOT NULL,
	tasks                       INTEGER NOT NULL,
    pgraphs                     INTEGER NOT NULL,
	alpha                       NUMERIC(3,2) NOT NULL,
	beta                        NUMERIC(3,2) NOT NULL,
	pia_ms            			DOUBLE PRECISION,
	dcop_ms         			DOUBLE PRECISION,
	pia_tt      				DOUBLE PRECISION,
	dcop_tt     				DOUBLE PRECISION,  
    pia_scheduled_tasks         INTEGER,
    dcop_scheduled_tasks        INTEGER,
	last_updated				TIMESTAMP,				

	PRIMARY KEY (_id)
);
   
