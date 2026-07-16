@echo off
REM Build and run the cart-pole sliding mode control example
cd /d %~dp0\..
mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp exec:java
