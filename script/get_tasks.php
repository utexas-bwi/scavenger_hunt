<?php
// connect to SQL database
include_once 'connect.php';
include_once 'auth.php';
$dbh = connect();
$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
try {
	// get the current hunt tasks from the database
	$query = "SELECT * FROM hunt_instructions_table";
	$stmt = $dbh->query($query);
	$stmt->setFetchMode(PDO::FETCH_ASSOC);
	// convert to XML
	$xml = new XMLWriter();

	$xml->openURI("php://output");
	$xml->startDocument();
	$xml->setIndent(true);

	$xml->startElement('tasks');

	while ($row = $stmt->fetch()) {
		$xml->startElement('task');
		$xml->writeAttribute('hunt_id', $row['hunt_id']);
		//$xml->writeAttribute('hunt_instr_id', $row['hunt_instr_id']);

		$xml->startElement('task_type');
		$xml->writeRaw($row['task_type']);
		$xml->endElement();

		$xml->startElement('params');
		$xml->writeRaw($row['param_value']);
		$xml->endElement();

		$xml->endElement();
	}
	$stmt = null;
	$xml->endElement();
	$xml->flush();

	// send back to robot
	header('Content-type: text/xml');
	echo($xml);
} catch (PDOException $e) {
	die($e->getMessage());
}
$dbh = null;
?>
