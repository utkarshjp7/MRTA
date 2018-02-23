\connect mrta;

SELECT 
    robots,
    tasks,  
    round(CAST(avg(ms1) as NUMERIC), 4) AS MS1_AVG, 
    round(CAST(avg(ms2) as NUMERIC), 4) AS MS2_AVG, 
    round(CAST(avg(tt1) as NUMERIC), 4) AS TT1_AVG,
    round(CAST(avg(tt2) as NUMERIC), 4) AS TT2_AVG,
    sum(scheduled_tasks1) AS TOTAL_TASKS1, 
    sum(scheduled_tasks2) AS TOTAL_TASKS2
FROM results 
GROUP BY robots, tasks
ORDER BY robots, tasks;