INSERT INTO `user_table` (3, 'test', 'pwd')
SELECT * FROM `user_table` 
WHERE NOT EXISTS (SELECT * FROM `user_table` WHERE user_id=3) 
LIMIT 1;