<?php
// connect to SQL database
include_once 'connect.php';
include_once 'auth.php';
$dbh = connect();
$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
try {
	// get the current hunt tasks from the database
	$query = "SELECT DISTINCT hunt_id FROM hunt_instructions_table";
	$stmt = $dbh->query($query);
	$stmt->setFetchMode(PDO::FETCH_ASSOC);
	// convert to XML
	$xml = new XMLWriter();

	$xml->openURI("php://output");
	$xml->startDocument();
	$xml->setIndent(true);

	while ($huntId = $stmt->fetch()) {
    $huntName = $huntId['hunt_id'];
    $query = "SELECT * FROM hunt_instructions_table WHERE hunt_id = $huntName";
	  $newStmt = $dbh->query($query);
    $newStmt->setFetchMode(PDO::FETCH_ASSOC);

    $xml->startElement('hunt');
    $xml->writeAttribute('name', $huntName);

    while($row = $newStmt -> fetch()){
      $task_type = $row['task_type'];
      $taskTable = $dbh -> query("SELECT * FROM task_table WHERE task_type = '$task_type'");
      $taskTable->setFetchMode(PDO::FETCH_ASSOC);
      $task = $taskTable->fetch();

		  //$xml->writeAttribute('hunt_instr_id', $row['hunt_instr_id']);

		  $xml->startElement('task');
		  $xml->writeAttribute('name', $row[task_type]);
		  $xml->writeAttribute('description', $task[description]);
		  $xml->writeAttribute('proof_format', $task[proof_type]);
      $xml->writeAttribute('proof_description', "");
		  $xml->writeAttribute('points', $task[score]);
		  $xml->writeAttribute('id', $task[id]);

		  $xml->startElement('parameters');
		  $xml->writeAttribute('name', $task[param_name]);
		  $xml->writeAttribute('value', $row[param_value]);
      $xml->endElement();
      
		  $xml->endElement();
    }

    $xml->endElement();
  }
	$xml->flush();

	// send back to robot
	header('Content-type: text/xml');
	echo($xml);
} catch (PDOException $e) {
	die($e->getMessage());
}
$dbh = null;
?>
