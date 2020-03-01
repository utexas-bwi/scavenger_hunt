<?php
function row($data) {
    $id = $data['id'];
    $task = $data['task_type'];
    $param = $data['param_name'];
    $proof = $data['proof_type'];
    $score = $data['score'];
    $descr = $data['description'];
    include 'components/task-table-row.html';
}

// Create connection
include 'connect.php';
$dbh = connect();

try {
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    $stmt = $dbh->query("SELECT * FROM task_table");
    $stmt->setFetchMode(PDO::FETCH_ASSOC);
    while ($data = $stmt->fetch()):
        row($data);
    endwhile;
    $stmt = null; 
} catch (PDOException $e) {
        die($e->getMessage());
}

// Close connection
$dbh = null;
?>
