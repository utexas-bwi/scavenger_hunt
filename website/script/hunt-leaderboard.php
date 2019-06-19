
<?php
  include_once 'connect.php';
  $dbh = connect();
  
  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  $dbh -> prepare ("DELETE FROM hunt_completed_table") -> execute();
  
  include '../script/completed_hunts.php';

  // display leaderboard for each hunt
  $huntStmt = $dbh -> query ("SELECT DISTINCT hunt FROM hunt_completed_table WHERE time != 0");
  $huntStmt -> setFetchMode(PDO::FETCH_ASSOC);

  while($huntList = $huntStmt -> fetch()){
    echo '<div class="content">
          <section id="main" class = "full">
          <div id="content">';
    // print out hunt name
    $hunt = $huntList['hunt'];
    echo '<h2>' .$hunt. '</h2>';
    // top of table
    echo '<div class="leaderboard flex column wrap leaderboard-header column grow leaderboard-table">
            <div class="leaderboard-row flex align-center row--header" style="border-radius: 0 !important;">
              <div class="row-user--header">Rank</div>
              <div class="row-collapse flex align-center">
                <div class="row-user--header" style = "padding-left: 4%">University</div>
                <div class="row-user--header" style = "padding-left: 14%">Attempts</div>
                <div class="row-user--header" style = "padding-left: 9%">Fastest Time(sec)</div>
                <div class="row-user--header" style = "padding-left: 4%">High Score</div>
              </div>
            </div>';
    
    //print out the results for each university, ranked by highest score
    $numUni = $dbh -> query("SELECT COUNT(DISTINCT university) FROM hunt_completed_table WHERE time != 0") ->fetchColumn();
    $stmt = $dbh -> query("SELECT * FROM hunt_completed_table WHERE time != 0 ORDER BY score DESC");
    $stmt->setFetchMode(PDO::FETCH_ASSOC);

    echo '<div class="leaderboard-body flex column grow">';   

    $arrayUniversity = array();

    for($x = 1; $x <= $numUni; $x++){
      
      //color of row
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

      //university name
      $university = $stmt -> fetch();
      $uni = $university['university'];

      while(in_array($uni, $arrayUniversity)){
        $university = $stmt -> fetch();
        $uni = $university['university'];
      }

      echo '<div class="row-caller flex">';
      if($uni == "University of Texas at Austin")
        echo '<img class="avatar" src="../../public_html/images/ut-logo.png" />';
      else if($uni == "University of Pennsylvania")
        echo '<img class="avatar" src="../../public_html/images/upenn-logo.png" />';
      else if($uni == "University of Michigan")
        echo '<img class="avatar" src="../../public_html/images/mich-logo.png" />';
      else if($uni == "University of Southern California")
        echo '<img class="avatar" src="../../public_html/images/usc-logo.png" />';
      else if($uni == "Brown University")
        echo '<img class="avatar" src="../../public_html/images/brown-logo.png" />'; 
      echo '<div class="row-user">';
      echo $university['university'];
      echo '</div> </div>';

      // attempts
      $numAttempts = $dbh -> query("SELECT COUNT(*) FROM hunt_completed_table WHERE university = '$uni' AND hunt = '$hunt'") -> fetchColumn();
      echo '<div class = "row-team">';
      echo $numAttempts;
      echo '</div>';

      // fastest time
      $getTime = $dbh -> query("SELECT * FROM hunt_completed_table WHERE university = '$uni' AND time != 0 ORDER BY time ASC");
      $getTime -> setFetchMode(PDO::FETCH_ASSOC);
      $time = ($getTime -> fetch())['time'];
      echo '<div class="row-rank">'.$time.'</div>';

      // high score
      $getScore = $dbh -> query("SELECT * FROM hunt_completed_table WHERE university = '$uni' AND time != 0 ORDER BY score DESC ");
      $getScore -> setFetchMode(PDO::FETCH_ASSOC);
      $score = ($getScore -> fetch())['score'];
      echo '<div class="row-calls">'.$score.'</div>';
      
    }
    echo '</div> </div>';
    echo '</div> </div>';
    
    echo '</div>'; 

    echo '</div></section></div>';
  }
?>
