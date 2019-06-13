<?php
  include 'connect.php';
  $dbh = connect();

  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  // sets correct to be true
  $sql = "UPDATE proof_table SET correct = 1 WHERE proof_id = ?";
  $stmt = $dbh->prepare($sql);
  $stmt->execute([$_POST['proofId']]);

  //obtains the current points of the user
  $uploaderId = $_POST['uploaderId'];
  $query = "SELECT score FROM user_table WHERE user_id = $uploaderId";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);
  $item = $stmt -> fetch();
  $curPoints = $item['score'];

  //obtains the points associated with the task accomplished
  $huntInstrId = $_POST['huntInstrId'];

  $query = "SELECT task_type FROM hunt_instructions_table WHERE hunt_instr_id = $huntInstrId";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);
  $item = $stmt -> fetch();
  $task_type = $item['task_type'];

  $query = "SELECT score FROM task_table WHERE task_type = '$task_type'";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);
  $item = $stmt -> fetch();
  $points = $item['score'];

  //updates points for the user
  $sql = "UPDATE user_table SET score = ? WHERE user_id = ?";
  $newPoints = $points + $curPoints;
  $stmt = $dbh->prepare($sql);
  $stmt->execute([$newPoints, $uploaderId]);
?>
