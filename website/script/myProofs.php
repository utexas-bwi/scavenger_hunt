<?php
  include_once 'connect.php';
  $dbh = connect();

  $dbh -> setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  include '../script/completed_hunts.php';
  $email = $_POST["user_email"];

  $userStmt = $dbh -> query ("SELECT * FROM user_table WHERE email = '$email'");
  $userStmt -> setFetchMode(PDO::FETCH_ASSOC);
  $userId = ($userStmt -> fetch())['user_id'];

  $huntStmt = $dbh -> query ("SELECT DISTINCT hunt FROM hunt_completed_table WHERE user_id = $userId ORDER BY hunt DESC");
  $huntStmt -> setFetchMode(PDO::FETCH_ASSOC);
  $huntList = $huntStmt -> fetch();

  if(!$huntList){
    echo '<div class="content">
    <section id="main" class = "full">
    <div id="content">';
    echo "<p> You have not sent in any proofs. </p>";

    echo '</div></section></div>';
  }
  while($huntList){
    echo '<div class="content">
    <section id="main" class = "full">
    <div id="content">';

    // print out hunt name
    $hunt = $huntList['hunt'];
    echo '<h2>' .$hunt. '</h2>';

    // get all tasks for this hunt
    $huntIdStmt = $dbh -> query("SELECT * FROM hunt_table WHERE hunt_name = '$hunt'");
    $huntIdStmt -> setFetchMode(PDO::FETCH_ASSOC);
    $huntId = ($huntIdStmt -> fetch())['hunt_id'];
    $huntInstrStmt = $dbh -> query("SELECT * FROM hunt_instructions_table WHERE hunt_id = $huntId");
    $huntInstrStmt -> setFetchMode(PDO::FETCH_ASSOC);

    $huntInstrArray = array();
    while($huntInstr = $huntInstrStmt -> fetch()){
      array_push($huntInstrArray, $huntInstr['hunt_instr_id']);
    }
    $huntInstrList = join("','", $huntInstrArray);

    echo '<table style="width:100%" class="table">';
    echo '<tr>';
    echo '<th>Time Submitted</th>';
    echo '<th>Task</th>';
    echo '<th>Parameters</th>';
    echo '<th>Verified</th>';
    echo '<th>Correct</th>';
    echo '<th>File</th>';
    echo '</tr>';


    //print out proof files and whether or not they have been verified
    $proofStmt = $dbh -> query ("SELECT * FROM proof_table WHERE uploader_id = $userId AND hunt_instr_id IN ('$huntInstrList')");
    $proofStmt -> setFetchMode(PDO::FETCH_ASSOC);
    while($proof = $proofStmt -> fetch()){
      echo '<tr>';
      $filename = $proof['filename'];
      $correct = $proof['correct'];
      $verified = $proof['verified'];

      echo '<td>';
      echo substr($filename, 0, 10) . " " . substr($filename, 10, 8);
      echo '</td>';

      // display task and parameters
      $huntInstr =  $proof['hunt_instr_id'];
      $taskStmt = $dbh -> query("SELECT * FROM hunt_instructions_table WHERE hunt_instr_id = $huntInstr");
      $taskStmt -> setFetchMode(PDO::FETCH_ASSOC);
      $task = $taskStmt -> fetch();
      $taskName = $task['task_type'];
      echo '<td>';
      echo $taskName;
      echo '</td>';

      echo '<td>';
      $taskParam = $task['param_value'];
      echo $taskParam;
      echo '</td>';
      // display verification state
      echo '<td>';
      if ($verified)
        echo "Yes";
      else
        echo "No";
      echo '</td>';

      echo '<td>';
      if($verified){
        if($correct)
          echo "Yes";
        else
          echo "No";
      } else
        echo "Not Verified";
      echo '</td>';

      echo '<td>';
      echo '<a href="../proof/'.$filename.'">';
      echo "[proof]</a>";
      echo '</td>';

      echo '</tr>';
    }

    echo '</table>';
    echo '</div></section></div>';
    $huntList = $huntStmt -> fetch();
  }

?>
