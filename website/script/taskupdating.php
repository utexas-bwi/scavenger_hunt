<?php

// Create connection
include 'connect.php';
$dbh = connect();
$loadNewCount = $_POST['loadNewCount'];

//var loadCount = 2;
function row($data) {
    $id = $data['id'];
    $task = $data['task_type'];
    $param = $data['param_name'];
    $proof = $data['proof_type'];
    $score = $data['score'];
    $descr = $data['description'];
    include '../public_html/components/task-table-row.html';
}

try {
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    $stmt = $dbh->query("SELECT * FROM task_table limit $loadNewCount");
    $stmt->setFetchMode(PDO::FETCH_ASSOC);
   while ($data = $stmt->fetch()):
        row($data);
    endwhile;
    $stmt = null; 
   //$result = mysql_query("SELECT * FROM task_table");
   //$array = mysql_fetch_row($result);
   //echo json_encode($array);
} catch (PDOException $e) {
        die($e->getMessage());
}

// Close connection
$dbh = null;
?>
