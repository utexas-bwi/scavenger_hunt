<?php
    include 'connect.php';

    $dbh = connect();
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

    $stmt = $dbh -> query("SELECT * FROM hunt_table WHERE hunt_name LIKE '%Turing%' ORDER BY release_date DESC");
    $stmt -> setFetchMode(PDO::FETCH_ASSOC);
    $prevHunt = $stmt -> fetch();
    $num = substr($prevHunt['hunt_name'], 12);
    $newNum = $num + 1;

    $newHuntName = "Turing Hunt " . $newNum;
    $newStartDate = $prevHunt['end_date'];
    $newEndDate = date('Y-m-d', strtotime($newStartDate . '+7 day'));

    $update = $dbh -> query("INSERT INTO hunt_table VALUES(0, '$newHuntName', '$newStartDate', 1595603883, '$newEndDate')");
    $getHuntId = $dbh -> query("SELECT * FROM hunt_table where hunt_name = '$newHuntName'");
    $getHuntId -> setFetchMode(PDO::FETCH_ASSOC);
    $huntId = ($getHuntId -> fetch())['hunt_id'];

    // update tasks for the hunt
    $rand = "SELECT * FROM task_table WHERE score > 300 ORDER BY rand()";
    
    // values of hunt_instructions_table = hunt_id, hunt_instr_id, task_type, parma_val
    $insert = "INSERT INTO hunt_instructions_table VALUES (?, 0, ?, ?)";

    for($x = 0; $x < 5; $x++){
        $stmt = $dbh -> query($rand);
        $stmt -> setFetchMode(PDO::FETCH_ASSOC);
        $task = $stmt -> fetch();
        $taskName = $task['task_type'];
        $paramName = $task['param_name'];

        $stmt = $dbh -> query("SELECT possible_values FROM task_param_table WHERE param_name = '$paramName'");
        $stmt->setFetchMode(PDO::FETCH_ASSOC);
        $values = [];
        $values = explode(", ", ($stmt->fetch())['possible_values']);
        $randNum = rand(0, count($values));
        $paramVal = $values[0];

        $dbh -> prepare($insert) -> execute([$huntId, $taskName, $paramVal]);
    }

?>  