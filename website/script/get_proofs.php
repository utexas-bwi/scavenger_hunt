<?php
// connect to SQL database
include_once 'connect.php';
// include_once 'auth.php';
$dbh = connect();
$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
try {
	global $dom, $node, $parnode;
  // get the current hunt tasks from the database
    $hunt_instr_id = $_POST['hunt_instr_id'];
    $user_id = $_POST['user_id'];
	$query = "SELECT * FROM proof_table WHERE hunt_instr_id = $hunt_instr_id AND uploader_id = $user_id";
	$stmt = $dbh->query($query);
	$stmt->setFetchMode(PDO::FETCH_ASSOC);
	// convert to XML
	$xml = new XMLWriter();

	$xml->openURI("php://output");
	$xml->startDocument();
    $xml->setIndent(true);
    
    $getTask = $dbh -> query("SELECT * FROM hunt_instructions_table WHERE hunt_instr_id = $hunt_instr_id");
    $getTask -> setFetchMode(PDO::FETCH_ASSOC);
    $task = $getTask -> fetch();
    $taskType = $task['task_type'];
    $paramValue = $task['param_value'];

    $xml->startElement('task');
    $xml->writeAttribute('type', $taskType);
    $xml->writeAttribute('parameter', $paramValue);

	while ($proofTable = $stmt->fetch()) {
        $correct = $proofTable['correct'];
        $filename = $proofTable['filename'];
        $xml->startElement('proof');
        $xml->writeAttribute('correct', $correct);
        $xml->writeAttribute('filename', "../proof/".$filename);
        $xml->endElement();
    }

    $xml->endElement();

	$xml->endDocument();
	$xml->flush();

	// send back to robot
	header('Content-type: text/xml');
	echo($xml);
	exit(0);
} catch (PDOException $e) {
	die($e->getMessage());
}
$dbh = null;
?>
