<?php
// connect to SQL database
include_once 'connect.php';
include_once 'auth.php';
$dbh = connect();
$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
try {
    global $dom, $node, $parnode;
    // get the current task from the database
    // $hunt = $_POST['hunt_name'];
    // $task = $_POST['task_name'];
    // $param = $_POST['param'];
    // $user_email = $_POST['user_email'];
    $user_email = "stefandebruyn@utexas.edu";
    $hunt = "BWI Lab Hunt";
    $task = "Find Object";
    $param = "chair";

    $stmt = $dbh -> query("SELECT * FROM user_table WHERE email = '$user_email'");
    $stmt -> setFetchMode(PDO::FETCH_ASSOC);
    $user_id = ($stmt -> fetch())['user_id'];

    $stmt = $dbh -> query("SELECT * FROM hunt_table WHERE hunt_name = '$hunt'");
    $stmt -> setFetchMode(PDO::FETCH_ASSOC);
    $hunt_id = ($stmt -> fetch())['hunt_id'];

    $stmt = $dbh -> query("SELECT * FROM hunt_instructions_table WHERE (task_type LIKE '%" . $task . "%' AND param_value LIKE '%". $param . "%')");

	// convert to XML
	$xml = new XMLWriter();

	$xml->openURI("php://output");
	$xml->startDocument();
    $xml->setIndent(true);

    while($taskList = $stmt -> fetch()){
        $hunt_instr_id = $taskList['hunt_instr_id'];
        $query = "SELECT * FROM proof_table WHERE hunt_instr_id = $hunt_instr_id AND uploader_id = $user_id" . " AND verified=1";
        $stmt = $dbh->query($query);
        $stmt->setFetchMode(PDO::FETCH_ASSOC);

        $getTask = $dbh -> query("SELECT * FROM hunt_instructions_table WHERE hunt_instr_id = $hunt_instr_id");
        $getTask -> setFetchMode(PDO::FETCH_ASSOC);
        $task = $getTask -> fetch();
        $taskType = $task['task_type'];
        $paramValue = $task['param_value'];

        $getFormat = $dbh -> query("SELECT * FROM task_table WHERE task_type = '$taskType'");
        $getFormat -> setFetchMode(PDO::FETCH_ASSOC);
        $proof_format = ($getFormat -> fetch())['proof_type'];

        $head_node = false;

        while ($proofTable = $stmt->fetch()) {
            if (!$head_node) {
              $head_node = true;
              $xml->startElement('task');
              $xml->writeAttribute('type', $taskType);
              $xml->writeAttribute('parameter', $paramValue);
              $xml->writeAttribute('proof_format', $proof_format);
              $xml->writeAttribute('hunt_instr_id', $hunt_instr_id);
            }

            $correct = $proofTable['correct'];
            $filename = $proofTable['filename'];
            $time = $proofTable['time_to_complete'];
            $xml->startElement('proof');
            $xml->writeAttribute('correct', $correct);
            $xml->writeAttribute('time', $time);
            $xml->writeAttribute('filename', $filename);
            $xml->endElement();
        }

        if ($head_node)
          $xml->endElement();
    }

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
