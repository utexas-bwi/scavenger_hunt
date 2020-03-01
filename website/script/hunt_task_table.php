<?php
function huntTaskTableRow($data) {
    $name = $data['name'];
    $params = $data['params'];
    include '../public_html/components/hunt-task-table-row.html'; 
}

function makeHuntTaskTable() {
	// Create connection
	include_once 'connect.php';
	include_once 'auth.php';
	$dbh = connect();
	try {
	    // get all task names and parameters	
	    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
	    $query = "SELECT task_type, param_name FROM task_table";
	    $stmt = $dbh->query($query);
	    $stmt->setFetchMode(PDO::FETCH_ASSOC);
	    while ($data = $stmt->fetch()):
		// get all possible values of parameters for task
		$query = "SELECT possible_values FROM task_param_table where param_name='" . $data['param_name'] . "'";
		$stmt2 = $dbh->query($query);
		$stmt2->setFetchMode(PDO::FETCH_ASSOC);
		// set up array to pass to row function
		$array = [];
		$array['name'] = $data['task_type'];
		$array['params'] = ($stmt2->fetch())['possible_values'];
		huntTaskTableRow($array);
	    endwhile;
	     $stmt = null; 
	} catch (PDOException $e) {
		die($e->getMessage());
	}

	// Close connection
	$dbh = null;
}
?>
