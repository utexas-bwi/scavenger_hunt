# scavenger\_hunt

## Packages Needed

* php
* mysql-server
* php-curl
* php-xml
* apache2
* libapache2-mod-php7.0
* php7.0-mysql

## Linux Configuration

When configuring apache2, to allow PHP to be housed within a .html file, you will have to add the following line to /etc/apache2/apache2.conf:

`AddHandler application/x-httpd-php .htm .html`

You will also have to configure PHP to use the PDO driver for connecting to the SQL database by editing /etc/php/7.0/apache2/php.ini. Find the following line and uncomment it:

`extension=php_pdo_mysql.dll`

After changing this line, you will have to restart apache2 for the changes to take effect:

`sudo service apache2 restart`

## Directory Structure

* public\_html: contains all website files
    * css: contains stylesheets
    * images: contains all images    
* script: contains php and javascript scripts used on the webpage

## Configuring MySQL 8.0 Authentication

After a fresh install of MySQL 8.0, you'll need to add a server account that the website will use to access the database. The user must be called `bwi` and have the bwilab sudo password:

```
sudo service mysql start
sudo mysql -u root
CREATE USER 'bwi'@'localhost'
  IDENTIFIED BY '[sudo password]';
GRANT ALL
  ON *.*
  TO 'bwi'@'localhost'
  WITH GRANT OPTION;
EXIT;
```

Then, initialize the database with the `.sql` file in the root of this repository:

```
sudo mysql -u bwi -p
CREATE DATABASE scavenger_hunt;
EXIT;
sudo mysql < scavenger_hunt.sql
```

Changes in MySQL 8.0 cause authentication errors when connecting via PHP if the server is not configured correctly. To fix this, locate the MySQL config file `/etc/mysql/my.cnf` and add the following lines:

```
[mysqld]
default-authentication-plugin=mysql_native_password
```

Then, restart MySQL:

`sudo service mysql restart`

## Viewing the Website

The website is currently set up on Ozil, Kane, and Rodriguez. It is also technically set up on Kif, but that machine is very old and has lots of scrambled privileges. Avoid headaches; don't use Kif.

To view the website, point your browser at `localhost`. You can also view it from a different machine at `localhost:8080` if you first SSH:

```
ssh -L 8080:128.83.143.224:80 bwilab@[ozil/kane/rodriguez/kif].csres.utexas.edu
```

When working on the site, you'll need to copy the repository's contents into `/var/www/html` on the host machine to see your changes propogated:

```
cd scavenger_hunt
sudo cp -r * /var/www/html
```
