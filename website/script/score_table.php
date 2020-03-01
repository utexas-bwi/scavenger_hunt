<?php
  include_once 'connect.php';
  $dbh = connect();

  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  // restart leaderboard
  $stmt = $dbh -> prepare("DELETE FROM score_table") -> execute();

  // score_table: university, score, num_verified, num_correct
  $insert = "INSERT INTO score_table VALUES (?, ?, ?, ?)";
  $modify = "UPDATE score_table SET score = ?, num_verified = ?, num_correct=? WHERE university = ?";

  //update the score_table
  $query = "SELECT * FROM hunt_completed_table WHERE (hunt like '%Dijkstra%' or hunt like '%Turing%')";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);

  while($hunt = $stmt -> fetch()){
    $numVerified = $hunt['num_verified'];
    $numCorrect = $hunt['num_correct'];
    $university = $hunt['university'];
    $userScore = $hunt['score'];

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