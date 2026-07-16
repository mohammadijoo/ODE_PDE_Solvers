@echo off
REM Build and run the 2D heat equation example
cd /d %~dp0\..
mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.heat2d.Heat2DApp exec:java
