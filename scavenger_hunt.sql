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
-- Current Database: `scavenger_hunt`
--

CREATE DATABASE /*!32312 IF NOT EXISTS*/ `scavenger_hunt` /*!40100 DEFAULT CHARACTER SET latin1 */;

USE `scavenger_hunt`;

--
-- Table structure for table `hunt_instructions_table`
--

DROP TABLE IF EXISTS `hunt_instructions_table`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `hunt_instructions_table` (
  `hunt_id` int(11) DEFAULT NULL,
  `hunt_instr_id` int(11) NOT NULL AUTO_INCREMENT,
  `task_type` varchar(20) DEFAULT NULL,
  `param_value` varchar(200) DEFAULT NULL,
  PRIMARY KEY (`hunt_instr_id`),
  KEY `hunt_id` (`hunt_id`),
  KEY `task_type` (`task_type`),
  CONSTRAINT `hunt_instructions_table_ibfk_1` FOREIGN KEY (`hunt_id`) REFERENCES `hunt_table` (`hunt_id`),
  CONSTRAINT `hunt_instructions_table_ibfk_2` FOREIGN KEY (`task_type`) REFERENCES `task_table` (`task_type`)
) ENGINE=InnoDB AUTO_INCREMENT=44 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `hunt_instructions_table`
--

LOCK TABLES `hunt_instructions_table` WRITE;
/*!40000 ALTER TABLE `hunt_instructions_table` DISABLE KEYS */;
INSERT INTO `hunt_instructions_table` VALUES (1,5,'Color Shirt','yellow'),(1,8,'Color Shirt','red'),(1,28,'Target Search','can'),(1,29,'Object Delivery',''),(2,30,'Target Search','can'),(2,31,'Human Following',''),(2,32,'Color Shirt','red'),(2,33,'Object Delivery',''),(2,34,'Human Following',''),(2,35,'Target Search','shoe'),(2,36,'Color Shirt','red'),(3,42,'Object Delivery',''),(3,43,'Human Following','');
/*!40000 ALTER TABLE `hunt_instructions_table` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `hunt_table`
--

DROP TABLE IF EXISTS `hunt_table`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `hunt_table` (
  `hunt_id` int(11) NOT NULL AUTO_INCREMENT,
  `hunt_name` varchar(255) DEFAULT NULL,
  `release_date` date DEFAULT NULL,
  `user_id` int(11) DEFAULT NULL,
  PRIMARY KEY (`hunt_id`),
  KEY `user_id` (`user_id`),
  CONSTRAINT `hunt_table_ibfk_1` FOREIGN KEY (`user_id`) REFERENCES `user_table` (`user_id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `hunt_table`
--

LOCK TABLES `hunt_table` WRITE;
/*!40000 ALTER TABLE `hunt_table` DISABLE KEYS */;
INSERT INTO `hunt_table` VALUES (1,'Hunt Name','2019-01-01',1),(2,'A Second Hunt','2019-04-10',1),(3,'New Hunt Test','2019-04-10',1);
/*!40000 ALTER TABLE `hunt_table` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task_param_table`
--

DROP TABLE IF EXISTS `task_param_table`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `task_param_table` (
  `param_id` int(11) NOT NULL AUTO_INCREMENT,
  `param_name` varchar(255) DEFAULT NULL,
  `possible_values` varchar(200) DEFAULT NULL,
  PRIMARY KEY (`param_id`),
  UNIQUE KEY `param_name` (`param_name`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task_param_table`
--

LOCK TABLES `task_param_table` WRITE;
/*!40000 ALTER TABLE `task_param_table` DISABLE KEYS */;
INSERT INTO `task_param_table` VALUES (1,'object','can, pencil, shoe'),(2,'color','yellow, red, blue, green'),(3,'','');
/*!40000 ALTER TABLE `task_param_table` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task_table`
--

DROP TABLE IF EXISTS `task_table`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `task_table` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `task_type` varchar(20) DEFAULT NULL,
  `param_name` varchar(255) DEFAULT NULL,
  `proof_type` varchar(200) DEFAULT NULL,
  `score` int(11) DEFAULT NULL,
  `description` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `task_type` (`task_type`),
  KEY `param_name` (`param_name`),
  CONSTRAINT `task_table_ibfk_1` FOREIGN KEY (`param_name`) REFERENCES `task_param_table` (`param_name`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=latin1;
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
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task_table_backup`
--

LOCK TABLES `task_table_backup` WRITE;
/*!40000 ALTER TABLE `task_table_backup` DISABLE KEYS */;
INSERT INTO `task_table_backup` VALUES (1,'Color Shirt','color','picture (jpg) of the person wearing the shirt',100,'Take a picture of a person who is wearing a specific color shirt.'),(2,'Target Search','object','picture (jpg) of the specified object',100,'Take a picture of the specified object.'),(3,'Human Following','','picture (jpg) of the human and picture (jpg) of the trajectories walked by the human and robot',100,'Follow a human for a certain distance.'),(4,'Object Delivery','','time-stamped description of object by person who gives the task and time-stamped description of object by person at ending location',150,'Find a person who is willing to specify an object to deliver from one location to another. Objects and locations can be selected through the robot UI in real time. Once the robot delivers the object, it should find a person at the ending location who can describe the object.');
/*!40000 ALTER TABLE `task_table_backup` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `user_table`
--

DROP TABLE IF EXISTS `user_table`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `user_table` (
  `user_id` int(11) NOT NULL AUTO_INCREMENT,
  `email` varchar(255) DEFAULT NULL,
  `university` varchar(255) DEFAULT NULL,
  `pass_hash` int(11) NOT NULL,
  PRIMARY KEY (`user_id`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Table structure for table `proof_table`
--

DROP TABLE IF EXISTS `proof_table`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `proof_table` (
  `proof_id` int(11) NOT NULL AUTO_INCREMENT,
  `uploader_id` int(11) NOT NULL,
  `filename` varchar(255) NOT NULL,
  `hunt_instr_id` int(11) NOT NULL,
  `verified` tinyint(1) NOT NULL DEFAULT 0,
  `correct` tinyint(1) NOT NULL DEFAULT 0,
  PRIMARY KEY (`proof_id`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `user_table`
--

LOCK TABLES `user_table` WRITE;
/*!40000 ALTER TABLE `user_table` DISABLE KEYS */;
/*!40000 ALTER TABLE `user_table` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2019-04-15 12:41:16
