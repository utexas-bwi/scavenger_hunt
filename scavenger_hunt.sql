-- MySQL dump 10.13  Distrib 5.7.25, for Linux (x86_64)
--
-- Host: localhost    Database: scavenger_hunt
-- ------------------------------------------------------
-- Server version	5.7.25-0ubuntu0.16.04.2

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `task_table`
--

DROP TABLE IF EXISTS `task_table`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `task_table` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `task_type` varchar(20) DEFAULT NULL,
  `param_name` set('color','object') DEFAULT NULL,
  `proof_type` varchar(200) DEFAULT NULL,
  `score` int(11) DEFAULT NULL,
  `description` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=7 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task_table`
--

LOCK TABLES `task_table` WRITE;
/*!40000 ALTER TABLE `task_table` DISABLE KEYS */;
INSERT INTO `task_table` VALUES (1,'Color Shirt','color','picture (jpg) of the person wearing the shirt',100,'Take a picture of a person who is wearing a specific color shirt.'),(2,'Target Search','object','picture (jpg) of the specified object',100,'Take a picture of the specified object.'),(3,'Human Following','','picture (jpg) of the human and picture (jpg) of the trajectories walked by the human and robot',100,'Follow a human for a certain distance.'),(4,'Object Delivery','','time-stamped description of object by person who gives the task and time-stamped description of object by person at ending location',150,'Find a person who is willing to specify an object to deliver from one location to another. Objects and locations can be selected through the robot UI in real time. Once the robot delivers the object, it should find a person at the ending location who can describe the object.');
/*!40000 ALTER TABLE `task_table` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task_table_backup`
--

DROP TABLE IF EXISTS `task_table_backup`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `task_table_backup` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `task_type` varchar(20) DEFAULT NULL,
  `param_name` set('color','object') DEFAULT NULL,
  `proof_type` varchar(200) DEFAULT NULL,
  `score` int(11) DEFAULT NULL,
  `description` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task_table_backup`
--

LOCK TABLES `task_table_backup` WRITE;
/*!40000 ALTER TABLE `task_table_backup` DISABLE KEYS */;
INSERT INTO `task_table_backup` VALUES (1,'Color Shirt','color','picture (jpg) of the person wearing the shirt',100,'Take a picture of a person who is wearing a specific color shirt.'),(2,'Target Search','object','picture (jpg) of the specified object',100,'Take a picture of the specified object.'),(3,'Human Following','','picture (jpg) of the human and picture (jpg) of the trajectories walked by the human and robot',100,'Follow a human for a certain distance.'),(4,'Object Delivery','','time-stamped description of object by person who gives the task and time-stamped description of object by person at ending location',150,'Find a person who is willing to specify an object to deliver from one location to another. Objects and locations can be selected through the robot UI in real time. Once the robot delivers the object, it should find a person at the ending location who can describe the object.');
/*!40000 ALTER TABLE `task_table_backup` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2019-03-13 17:46:11
