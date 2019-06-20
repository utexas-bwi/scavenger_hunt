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

    //print out proof files and whether or not they have been verified
    $proofStmt = $dbh -> query ("SELECT * FROM proof_table WHERE uploader_id = $userId AND hunt_instr_id IN ('$huntInstrList')");
    $proofStmt -> setFetchMode(PDO::FETCH_ASSOC);
    while($proof = $proofStmt -> fetch()){
      $filename = $proof['filename'];
      $correct = $proof['correct'];
      $verified = $proof['verified'];
      echo "<p>";
      echo '<a href="../proof/'.$filename.'">';
      echo $filename."</a>";


      // display task and parameters
      echo '<span style="font-weight:bold"> Task: </span>';
      $huntInstr =  $proof['hunt_instr_id'];
      $taskStmt = $dbh -> query("SELECT * FROM hunt_instructions_table WHERE hunt_instr_id = $huntInstr");
      $taskStmt -> setFetchMode(PDO::FETCH_ASSOC);
      $task = $taskStmt -> fetch();
      $taskName = $task['task_type'];
      echo $taskName;

      $taskParam = $task['param_value'];
      if($taskParam != ""){
        echo '<span style="font-weight:bold"> Parameters: </span>';
        echo $taskParam;
      }

      // display verification state
      echo '<span style="font-weight:bold"> Verified: </span>';
      if ($verified)
        echo "Yes";
      else 
        echo "No";
      echo '<span style="font-weight:bold"> Correct: </span>';
      if($verified){
        if($correct)
          echo "Yes";
        else 
          echo "No";
      } else 
        echo "Not Verified";


      echo "</p>";
    }
    echo '</div></section></div>';  
    $huntList = $huntStmt -> fetch();
  }

?>