\connect mrta;

SELECT 
	dcop.robots,
	dcop.tasks,
	dcop_ms,
	dcop_collab_ms,
	dcop_idle,
	dcop_collab_idle,
	dcop_tt,
	dcop_collab_tt,
	dcop_tasks,
	dcop_collab_tasks

FROM
	(SELECT 
	    robots,
	    tasks, 
	    round(cast(min(ms1) as numeric), 4) as dcop_ms,
	    round(cast(min(ms2) as numeric), 4) as dcop_collab_ms,
	    round(cast(min(tt1) as numeric), 4) as dcop_tt,
	    round(cast(min(tt2) as numeric), 4) as dcop_collab_tt,
	    round(cast(min(it1) as numeric), 4) as dcop_idle,
	    round(cast(min(it2) as numeric), 4) as dcop_collab_idle,
	    sum(scheduled_tasks1) as dcop_tasks,
	    sum(scheduled_tasks2) as dcop_collab_tasks
	FROM results
	WHERE comment = '040918_dcop'
	GROUP BY robots, tasks) dcop
/*
JOIN
	(SELECT 
	    robots,
	    tasks, 
	    round(cast(min(ms1) as numeric), 4) as pia_ms,
	    round(cast(min(tt1) as numeric), 4) as pia_tt,
            sum(scheduled_tasks1) as pia_tasks
	FROM results
	WHERE comment = '040218_pia'
	GROUP BY robots, tasks) as pia

ON dcop.robots = pia.robots AND dcop.tasks = pia.tasks
*/
order by  robots, tasks;

