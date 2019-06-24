<?php
  include_once 'connect.php';
  $dbh = connect();

  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  // restart leaderboard
  $delete = "DELETE FROM score_table";
  $stmt = $dbh -> prepare($delete);
  $stmt -> execute();

  //update the score_table
  $query = "SELECT * FROM user_table";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);

  // score_table: university, score, num_verified, num_correct
  $insert = "INSERT INTO score_table VALUES (?, ?, ?, ?)";
  $modify = "UPDATE score_table SET score = ?, num_verified = ?, num_correct=? WHERE university = ?";

  while($user = $stmt -> fetch()){
    $userId = $user['user_id'];
    $numVerified = 0;
    $numCorrect = 0;
    $query2 = "SELECT * FROM proof_table WHERE uploader_id = $userId";
    $getPercent = $dbh -> query($query2);
    $getPercent -> setFetchMode(PDO::FETCH_ASSOC);
    while($proof = $getPercent -> fetch()){
      $numVerified = $dbh -> query("SELECT count(*) FROM proof_table WHERE verified = 1") -> fetchColumn();
      $numCorrect = $dbh -> query("SELECT count(*) FROM proof_table WHERE correct = 1") -> fetchColumn();
    }
    $university = $user['university'];
    $userScore = $user['score'];
    $sql = "SELECT * FROM score_table WHERE university = '$university'";
    $scoreTable = $dbh->query($sql);
    $scoreTable->setFetchMode(PDO::FETCH_ASSOC);
    if($uni = $scoreTable -> fetch()){
      //modify current score
      $curScore = $uni['score'];
      $newScore = $curScore + $userScore;
      $scoreTable = $dbh->prepare($modify);
      $scoreTable->execute([$newScore, $uni['num_verified'] + $numVerified, $uni['num_correct'] + $numCorrect, $university]);
    } else {
      //insert new university
      $scoreTable = $dbh->prepare($insert);
      $scoreTable->execute([$university, $userScore, $numVerified, $numCorrect]);
    }
  }

?>