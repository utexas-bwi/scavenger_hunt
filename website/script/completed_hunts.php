<?php
  include_once 'connect.php';
  $dbh = connect();
  
  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  $dbh -> prepare ("DELETE FROM hunt_completed_table") -> execute();

  // create a "hunt completed" table that talllies up the time and score for each hunt that 
  // a university has completed
  // a university may have completed a hunt more than one time
  $insert = "INSERT INTO hunt_completed_table VALUES (?, ?, ?, ?, ?)";

  $userTable = $dbh -> query("SELECT * FROM user_table");
  $userTable -> setFetchMode(PDO::FETCH_ASSOC);
  while($user = $userTable -> fetch()){
    $userId = $user['user_id'];

    $university = $user['university'];

    $proofTable = $dbh -> query("SELECT * FROM proof_table WHERE uploader_id = " .$userId);
    $proofTable -> setFetchMode(PDO::FETCH_ASSOC);
    $proof = $proofTable -> fetch();

    while($proof){

      $arrayHuntInstr = array();
      $arrayProofId = array();

      $huntInstrId = $proof['hunt_instr_id'];
      $huntInstrTable = $dbh -> query ("SELECT * FROM hunt_instructions_table WHERE hunt_instr_id = " .$huntInstrId);
      $huntInstrTable -> setFetchMode(PDO::FETCH_ASSOC);
      $huntId = ($huntInstrTable -> fetch())['hunt_id'];

      $huntTable = $dbh -> query ("SELECT * FROM hunt_table WHERE hunt_id = " .$huntId);
      $huntTable -> setFetchMode(PDO::FETCH_ASSOC);
      $hunt = ($huntTable -> fetch())['hunt_name'];

      $numTasks = $dbh -> query ("SELECT count(*) FROM hunt_instructions_table WHERE hunt_id = " .$huntId) -> fetchColumn();
      $taskNum = 0;

      while($taskNum < $numTasks && $proof){
        $huntInstrId = $proof['hunt_instr_id'];
        $proofId = $proof['proof_id'];
        $taskNum = $taskNum + 1;
        if(in_array($huntInstrId, $arrayHuntInstr)){
          $taskNum = $numTasks;
          break;
        }
        array_push($arrayHuntInstr, $huntInstrId);
        array_push($arrayProofId, $proofId);
        $proof = $proofTable -> fetch();
      }
      $time = 0;
      $score = 0;
      for($x = 0; $x < count($arrayProofId); $x++){
        $getProof =  $dbh -> query("SELECT * FROM proof_table WHERE proof_id = " .$arrayProofId[$x]);
        $getProof -> setFetchMode(PDO::FETCH_ASSOC);
        $taskProof = $getProof -> fetch();
        if($taskNum == $numTasks){
          $time += $taskProof['time_to_complete'];
        }
        if($proof['correct']){
          $getTask = $dbh -> query("SELECT * FROM hunt_instructions_table WHERE hunt_instr_id = " .$arrayHuntInstr[$x]);
          $getTask -> setFetchMode(PDO::FETCH_ASSOC);
          $task = ($getTask -> fetch())['task_type'];
          $getScore = $dbh -> query("SELECT * FROM task_table WHERE task_type = '$task'");
          $getScore -> setFetchMode(PDO::FETCH_ASSOC);
          $score += ($getScore -> fetch())['score'];
        }
      }
      $huntCompTable = $dbh-> prepare($insert);
      $huntCompTable->execute([$university, $hunt, $time, $score, $userId]);
    }
  }
?>