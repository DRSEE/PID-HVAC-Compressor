// Paramètres PID
double kp = 10.0, ki = 5.0, kd = 1.0; // Coefficients PID
double setpoint = 20.0;              // Température de consigne (fixe à 20°C)
double integral = 0.0;               // Accumulateur pour la partie intégrale
double prevError = 0.0;              // Erreur précédente pour la partie dérivée
double output = 0.0;                 // Sortie PID

// Plage de la sortie (vitesse en RPM)
const int minRPM = 1500;
const int maxRPM = 6000;

// Pins
const int potPin = A0;  // Potentiomètre pour la température mesurée
const int outputPin = 9; // Pin PWM pour le contrôle du compresseur

void setup() {
  Serial.begin(9600);      // Initialisation du port série
  pinMode(outputPin, OUTPUT); // Configurer la pin PWM comme sortie
}

void loop() {
  // Lire la température simulée avec le potentiomètre
  int potValue = analogRead(potPin);
  double measuredTemp = map(potValue, 0, 1023, 10, 40); // Température entre 10°C et 40°C

  // Calcul de l'erreur
  double error = measuredTemp - setpoint ;
  Serial.print("error : ");
  Serial.print(error);
    
  // Calcul de la partie proportionnelle
  double proportional = kp * error;
  Serial.print(" Prop: ");
  Serial.print(proportional);
  
  // Calcul de la partie intégrale
  integral += error * 0.1; // deltaTime = 0.1s (boucle de 100 ms)
  Serial.print(" Integral: ");
  Serial.print(integral);
  
  // Limiter la partie intégrale pour éviter l'accumulation excessive
  if (integral > 1000) integral = 1000;
  if (integral < -1000) integral = -1000;
  
  double integralPart = ki * integral;
  
  // Calcul de la partie dérivée
  double derivative = kd * (error - prevError) / 0.1; // deltaTime = 0.1s
  Serial.print(" derivative: ");
  Serial.print(derivative);
  
  // Somme des termes PID
  output = proportional + integralPart + derivative;
  Serial.print(" ");
  
  // Limiter la sortie pour rester dans la plage autorisée (RPM)
  if (output > maxRPM) output = maxRPM;
  if (output < minRPM) output = minRPM;

  // Générer le signal PWM correspondant à la vitesse
  int pwmValue = map(output, minRPM, maxRPM, 0, 255); // Convertir RPM en PWM (0-255)
  analogWrite(outputPin, pwmValue);

  // Stocker l'erreur pour le calcul suivant
  prevError = error;
  
  // Afficher les informations sur le moniteur série
  Serial.print("Consigne: ");
  Serial.print(setpoint);
  Serial.print("°C | Température mesurée: ");
  Serial.print(measuredTemp);
  Serial.print("°C | Vitesse: ");
  Serial.print(output);
  Serial.println(" RPM");
  
  delay(100); // Attendre 100 ms avant la prochaine itération
  
}
