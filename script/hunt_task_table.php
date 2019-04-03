<?php
function row($data) {
    $name = $data['name'];
    $params = $data['params'];
    include '../public_html/components/hunt-task-table-row.html'; 
}

// Create connection
include 'connect.php';
include 'auth.php';

$dbh = connect();
try {
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    $query = "SELECT task_type, param_name FROM task_table";
    $stmt = $dbh->query($query);
    $stmt->setFetchMode(PDO::FETCH_ASSOC);
    /*while ($data = $stmt->fetch()):
	$query = "SELECT possible_values FROM task_param_table where param_name='" . $data['param_name'] . "'";
	$stmt2 = $dbh->query($query);
	$stmt2->setFetchMode(PDO::FETCH_ASSOC);
	$array = {};
	$array['name'] = $data['task_type'];
	$array['params'] = ($stmt2->fetch())['possible_values'];
	row($array);
    endwhile;
     $stmt = null; 
} catch (PDOException $e) {
        die($e->getMessage());
}

// Close connection
$dbh = null;*/
?>
