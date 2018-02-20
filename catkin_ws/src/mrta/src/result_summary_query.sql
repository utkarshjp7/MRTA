\connect mrta;

SELECT 
    robots,
    tasks,  
    round(CAST(avg(pia_ms) as NUMERIC), 4) AS PIA_MS_AVG, 
    round(CAST(avg(dcop_ms) as NUMERIC), 4) AS DCOP_MS_AVG, 
    round(CAST(avg(pia_tt) as NUMERIC), 4) AS PIA_TT_AVG,
    round(CAST(avg(dcop_tt) as NUMERIC), 4) AS DCOP_TT_AVG,
    sum(pia_scheduled_tasks) AS PIA_TOTAL_TASKS, 
    sum(dcop_scheduled_tasks) AS DCOP_TOTAL_TASKS
FROM results 
GROUP BY robots, tasks
ORDER BY robots, tasks;