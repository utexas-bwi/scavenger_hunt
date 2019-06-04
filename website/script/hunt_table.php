<?php
include_once 'connect.php';

/* functions to handle table that stores the hunt instructions */

function huntTableRow($data) {
    $id = $data['hunt_instr_id'];
    $task = $data['task_type'];
    $param = $data['param_value'];
    include '../public_html/components/hunt-table-row.html'; 
}

function makeHuntTable() {
	if (isset($_GET['id'])) {
	    $id = $_GET['id'];
	    // Create connection
	    $dbh = connect();
	    try {
		$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
		$query = "SELECT * FROM hunt_instructions_table where hunt_id=" . $id;
		$stmt = $dbh->query($query);
		$stmt->setFetchMode(PDO::FETCH_ASSOC);
		while ($data = $stmt->fetch()):
		    huntTableRow($data);
	endwhile;
	$stmt = null; 
	    } catch (PDOException $e) {
		die($e->getMessage());
	    }

	    // Close connection
	    $dbh = null;
	}
}

/* functions to handle table that stores tasks to add to hunt */

function huntTaskTableRow($data) {
    $name = $data['name'];
    $params = $data['params'];
    include '../public_html/components/hunt-task-table-row.html'; 
}

function makeHuntTaskTable() {
	// Create connection
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

function getHuntName($id) {
    $dbh = connect(); 
    try {
	    // get all task names and parameters	
	    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
	    $query = "SELECT hunt_name FROM hunt_table WHERE hunt_id = " . $id;
		$stmt = $dbh->query($query);
		$stmt->setFetchMode(PDO::FETCH_ASSOC);
		$name = ($stmt->fetch())['hunt_name'];
		echo $name;
	} catch (PDOException $e) {
		die($e->getMessage());
	}
}

?>
