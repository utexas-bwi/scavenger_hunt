<?php
  include 'connect.php';
  $dbh = connect();
  
  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  // restart leaderboard (maybe better to update leaderboard when user's score is updated?)
  $delete = "DELETE FROM score_table";
  $stmt = $dbh -> prepare($delete);
  $stmt -> execute();

  //update the score_table
  $query = "SELECT * FROM user_table";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);

  // score_table: university, score, percent_success
  $insert = "INSERT INTO score_table VALUES (?, ?, ?)";
  $modify = "UPDATE score_table SET score = ? WHERE university = ?";
 
  while($user = $stmt -> fetch()){
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
      $scoreTable->execute([$newScore, $university]);
    } else {
      //insert new university
      $scoreTable = $dbh->prepare($insert);

      //TODO calculate percentage
      $scoreTable->execute([$university, $userScore, 0]);
    }
  }

  echo '<div class="leaderboard-body flex column grow">';

  //creates the table to be displayed on the page

  // counts the number of universities that have partaken in a hunt
  $sort = "SELECT count(*) FROM score_table ORDER BY score DESC";
  $numRows = $dbh->query($sort)->fetchColumn(); 

  $sort = "SELECT * FROM score_table ORDER BY score DESC";
  $stmt = $dbh->query($sort);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);
  
  for ($x = 1; $x <= $numRows; $x++) {
    
    if($x % 2 == 0){
      echo '<div class="leaderboard-row flex row-alt align-center">';
    } else {
      echo '<div class="leaderboard-row flex align-center">';
    }
    
    //position
    echo '<div class="row-position">';
    echo $x;
    echo '</div>';

    echo '<div class="row-collapse flex align-center">';
    
    $university = $stmt -> fetch();
    
    echo '<div class="row-caller flex">';
    // echo '<img class="avatar" src="../../public_html/images/ut-logo.png" />';
    echo '<div class="row-user">';
    echo $university['university'];
    echo '</div> </div>';
    
    echo '<div class="row-team">';
    echo 0;
    echo '</div>';

    echo '<div class="row-rank">';
    echo $university['percent_success'];
    echo '</div> </div>';

    echo '<div class="row-calls">';
    echo $university['score'];
    echo '</div> </div>';
  } 

  echo '</div>';
?>