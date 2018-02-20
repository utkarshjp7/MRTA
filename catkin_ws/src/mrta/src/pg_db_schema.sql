\connect mrta;

DROP TABLE results;

CREATE TABLE IF NOT EXISTS results (
	_id                         SERIAL NOT NULL,
	robots                      INTEGER NOT NULL,
	tasks                       INTEGER NOT NULL,
    pgraphs                     INTEGER NOT NULL,
	alpha                       NUMERIC(3,2) NOT NULL,
	beta                        NUMERIC(3,2) NOT NULL,
	pia_avg_makespan            DOUBLE PRECISION,
	pia_avg_time_travelled      DOUBLE PRECISION,
    pia_scheduled_tasks         INTEGER,
    dcop_avg_makespan           DOUBLE PRECISION,
    dcop_avg_time_travelled     DOUBLE PRECISION,  
    dcop_scheduled_tasks        INTEGER,

	PRIMARY KEY (_id)
);
   
