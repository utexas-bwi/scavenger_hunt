
<?php
  include_once 'connect.php';
  $dbh = connect();
  
  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  include 'completed_hunts.php';

  // display leaderboard for each hunt

  $huntStmt = $dbh -> query ("SELECT * FROM hunt_completed_table WHERE (hunt like '%Dijkstra%' or hunt like '%Turing%')");
  $huntStmt -> setFetchMode(PDO::FETCH_ASSOC);

  // mysql> select * from hunt_table where (hunt_name like "%Dijkstra%" or hunt_name like "%Turing%");


  while($huntList = $huntStmt -> fetch()){
    echo '<div class="content">
          <section id="main" class = "full">
          <div id="content">';
    // print out hunt name
    $huntName = $huntList['hunt'];
    echo '<h2>' .$huntName. '</h2>';
    // top of table
    echo '<div class="leaderboard flex column wrap leaderboard-header column grow leaderboard-table">
            <div class="leaderboard-row flex align-center row--header" style="border-radius: 0 !important;">
              <div class="row-user--header">Rank</div>
              <div class="row-collapse flex align-center">
                <div class="row-user--header" style = "padding-left: 2%">University</div>
                <div class="row-user--header" style = "padding-left: 16%">Tasks Attempted</div>
                <div class="row-user--header">Percent Success</div>
                <div class="row-user--header">Points</div>
              </div>
            </div>';
    
    echo '<div class="leaderboard-body flex column grow">';   
    
    //print out the results for each university, ranked by highest score
    $numRows = $dbh -> query("SELECT COUNT(DISTINCT university) FROM hunt_completed_table WHERE time != 0") ->fetchColumn();
    $stmt = $dbh -> query("SELECT * FROM hunt_completed_table WHERE hunt = '$huntName' ORDER BY score DESC");
    $stmt->setFetchMode(PDO::FETCH_ASSOC);

    include 'leaderboard.php';

    echo '</div> </div> </div> </div> </div> </section> </div>';
  }
?>
