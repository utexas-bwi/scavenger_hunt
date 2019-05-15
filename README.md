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

## Notes on Ozil

On a Linux machine (such as Ozil), you will have to copy the files from this repository into the /var/www/html folder on your computer to have it displayed on localhost. This folder requires sudo access to copy the files.

The mysql password is the same as the sudo password for the bwilab account on Ozil.

## Viewing the Website on Kane/Ozil

Currently, the website is hosted on Ozil, Kane, and Kif, so you will have to have a BWI account to view the website. You can view the website by ssh-ing into whichever of the three machines you wish with the following command:

`ssh -L 8080:128.83.143.224:80 [username]@[ozil/kane/kif].csres.utexas.edu`

Then, open a web browser and go to the website address `http://localhost:8080`, and you will be able to see the website.
