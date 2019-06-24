<?php
    include 'connect.php';

    $dbh = connect();
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

    $stmt = $dbh -> query("SELECT * FROM hunt_table WHERE hunt_name LIKE '%Dijkstra%' ORDER BY release_date DESC");
    $stmt -> setFetchMode(PDO::FETCH_ASSOC);
    $prevHunt = $stmt -> fetch();
    $num = substr($prevHunt['hunt_name'], 14);
    $newNum = $num + 1;

    $newHuntName = "Dijkstra Hunt " . $newNum;
    $newStartDate = $prevHunt['end_date'];
    $newEndDate = date('Y-m-d', strtotime($newStartDate . '+1 day'));

    

    $update = $dbh -> query("INSERT INTO hunt_table VALUES($newNum, '$newHuntName', '$newStartDate', -1589239765, '$newEndDate')");
    

?>