#include "Arduino_H7_Video.h"
#include "lvgl.h"
#include "Arduino_GigaDisplayTouch.h"
#include "SDRAM.h"
 
#define APP_VERSION "1.7"

static lv_obj_t* g_versionBadge = nullptr;

  enum ScreenId { // Identifiants des écrans
  SCR_ACCUEIL,
  SCR_SEL_CAN,
  SCR_VEHICULE,
  SCR_VEHICULE_TOTALE,
  SCR_VEHICULE_LECT_TOTALE,
  SCR_VEHICULE_SEL_AS,
  SCR_VEHICULE_LECT_AS,
  SCR_VEHICULE_PGN,
  SCR_VEHICULE_LECT_PGN,
  SCR_EQUIPEMENT,
  SCR_EQUIPEMENT_SEL_TOTALE,
  SCR_EQUIPEMENT_LECT_TOTALE,
  SCR_EQUIPEMENT_SEL_PGN,
  SCR_EQUIPEMENT_LECT_PGN,
  SCR_WIFI,
  SCR_MQTT
 };

// _______________  Partie BUSCAN ___________________

#ifndef ARDUINO_GIGA
  #error This sketch runs on an Arduino Giga R1 board
#endif

static const uint32_t FDCAN1_MESSAGE_RAM_WORD_SIZE = 0;
static const uint32_t FDCAN2_MESSAGE_RAM_WORD_SIZE = 2560;

#include <ACANFD_GIGA_R1.h>

extern ACANFD_GIGA_R1 fdcan2;
#define fdcan1 fdcan2 

enum class BusProfile : uint8_t { Equipement, Vehicule };
const int relaisSelectionBUSCAN = 22;
bool etatRelaisSelectionBUSCAN = false;

static void stopAllCapturesAndTimers();
static void endCAN_if_supported();
static void setRelayForCurrentSelection();
static void updateRelaisImageOnSelectionPage();


//############################ Sélection Baud rate + Serial ####################################

 // ===== Sélection et init des profils CAN + port série =====
 static inline uint32_t canBitrateFor(BusProfile p)    { return (p == BusProfile::Vehicule) ? 500000 : 250000; }
 static inline uint32_t serialBaudFor(BusProfile p)    { return (p == BusProfile::Vehicule) ? 500000 : 250000; }

 static void setRelayFor(BusProfile p) {  // HIGH = Véhicule, LOW = Equipement (comme tu l'avais)
  digitalWrite(relaisSelectionBUSCAN, (p == BusProfile::Vehicule) ? HIGH : LOW);
  delay(5); // stabilisation
 }

 static bool initCAN_for(BusProfile p) {
  const uint32_t nominal = (p == BusProfile::Vehicule) ? 500000 : 250000;

  ACANFD_GIGA_R1_Settings settings(
    nominal,
    DataBitRateFactor::x1,  // CAN classique (pas de data rate FD)
    1000
  );

  settings.mEnableRetransmission = false;   // sniff propre (optionnel)
  // settings.mSilent = true;               // si dispo dans ta version, pour listen-only

  const uint32_t err = fdcan1.beginFD(settings);
  if (err != 0) {
    Serial.print(F("[ACAN] beginFD error = 0x"));
    Serial.println(err, HEX);
    return false;
  }
  return true;
 }
 
 lv_obj_t *lblVehiculeBaud = nullptr, *lblVehiculeSerial = nullptr; // Pointeurs optionnels vers tes labels (déclare-les une fois en global)
 lv_obj_t *lblEquipBaud   = nullptr, *lblEquipSerial   = nullptr;

 unsigned long vitesseBaudRate = 0;
 unsigned long vitesseSerial = 0;

 static void updateSpeedLabels(BusProfile p) {
  char b1[48], b2[48];
  const uint32_t canB = (p == BusProfile::Vehicule) ? 500000 : 250000;
  const uint32_t serB = canB;

  vitesseBaudRate = canB;
  vitesseSerial = serB;

  snprintf(b1, sizeof(b1), "Vitesse en baud : %lu", (unsigned long)canB);
  snprintf(b2, sizeof(b2), "Vitesse port serie : %lu", (unsigned long)serB);

  if (objAlive(lblVehiculeBaud))   lv_label_set_text(lblVehiculeBaud,   b1);
  if (objAlive(lblVehiculeSerial)) lv_label_set_text(lblVehiculeSerial, b2);
  if (objAlive(lblEquipBaud))      lv_label_set_text(lblEquipBaud,      b1);
  if (objAlive(lblEquipSerial))    lv_label_set_text(lblEquipSerial,    b2);
 }


 static bool applyBusProfile(BusProfile p) {
  // 1) On coupe tout ce qui lit/touche au CAN et à la UI liée
  stopAllCapturesAndTimers();

  // 2) Met le relais et laisse se stabiliser
  setRelayFor(p);

  // 3) Arrête proprement le contrôleur CAN avant reinit
  endCAN_if_supported();

  // 4) (Re)configure le CAN
  if (!initCAN_for(p)) {
    // En cas d’échec, ne pas toucher au Serial pour garder les logs
    return false;
  }

  // 5) Serie : ÉVITE d’appeler end()/begin() sur l’USB CDC
  // Sur Giga R1, la vitesse USB est virtuelle -> inutile et parfois instable.
  // Si tu veux quand même changer le débit (par ex. pour un port UART HW),
  // encapsule derrière un #ifdef sur le port utilisé.
  // setSerialBaud(serialBaudFor(p));

  // 6) Mets à jour les étiquettes
  updateSpeedLabels(p);

  return true;
 }



// ______________________ Partie Wifi ________________________

#include <WiFi.h>
char ssid[33]     = "";  // vide; on affiche un placeholder dans la UI
char password[65] = "";
IPAddress ip;  

WiFiClient wifiClient;

 // ---- Effet visuel de clic pour le bouton Wi-Fi uniquement ----
static lv_style_t styleBtnBaseClick, styleBtnPressedGreen, styleBtnPressedRed;
static lv_style_transition_dsc_t trBtnClick;
static bool clickStylesInit = false;

static void initClickStyles() {
  if (clickStylesInit) return;

  static const lv_style_prop_t props[] = {
    LV_STYLE_TRANSLATE_Y, LV_STYLE_SHADOW_OFS_Y, LV_STYLE_BG_COLOR, LV_STYLE_PROP_INV
  };
  lv_style_transition_dsc_init(&trBtnClick, props, lv_anim_path_ease_out, 120, 0, NULL);

  // Style de base (commun)
  lv_style_init(&styleBtnBaseClick);
  lv_style_set_radius(&styleBtnBaseClick, 10);
  lv_style_set_shadow_width(&styleBtnBaseClick, 12);
  lv_style_set_shadow_ofs_y(&styleBtnBaseClick, 4);
  lv_style_set_bg_opa(&styleBtnBaseClick, LV_OPA_COVER);
  lv_style_set_transition(&styleBtnBaseClick, &trBtnClick);

  // Pressé vert (Se connecter)
  lv_style_init(&styleBtnPressedGreen);
  lv_style_set_translate_y(&styleBtnPressedGreen, 2);
  lv_style_set_shadow_ofs_y(&styleBtnPressedGreen, 2);
  lv_style_set_bg_color(&styleBtnPressedGreen, lv_color_hex(0x256628)); // vert foncé
  lv_style_set_transition(&styleBtnPressedGreen, &trBtnClick);

  // Pressé rouge (Deconnecter)
  lv_style_init(&styleBtnPressedRed);
  lv_style_set_translate_y(&styleBtnPressedRed, 2);
  lv_style_set_shadow_ofs_y(&styleBtnPressedRed, 2);
  lv_style_set_bg_color(&styleBtnPressedRed, lv_color_hex(0x9F1A1A)); // rouge foncé
  lv_style_set_transition(&styleBtnPressedRed, &trBtnClick);

  clickStylesInit = true;
}

 lv_obj_t *btnWifiToggle = nullptr;  // --- Bouton connexion/déconnexion Wi-Fi (toggle)

 static lv_style_t styleBtnDanger; 
 static bool styleBtnDanger_init = false;

 static void applyStyleBtnDanger(lv_obj_t *btn) {
  if (!styleBtnDanger_init) {
    lv_style_init(&styleBtnDanger);
    lv_style_set_bg_color(&styleBtnDanger, lv_color_hex(0xC62828));
    lv_style_set_bg_opa(&styleBtnDanger, LV_OPA_COVER);   // <<< AJOUT
    lv_style_set_radius(&styleBtnDanger, 10);
    lv_style_set_border_color(&styleBtnDanger, lv_color_hex(0x7F1D1D));
    lv_style_set_border_width(&styleBtnDanger, 2);
    styleBtnDanger_init = true;
  }
  lv_obj_add_style(btn, &styleBtnDanger, 0);
}

// ________________________________________  Partie PubSubClient  _____________________________________

#include <PubSubClient.h>

unsigned long dernierTempsConexionMQTT = 0;
const uint32_t intervalConnexionMQTT = 10000; 

static unsigned long lastSpeedPub = 0;
static const uint32_t SPEED_PUB_PERIOD = 5000; // 5 secondes

char mqtt_server[64]  = "";
char mqttUser[33]     = "";
char mqttPassword[65] = "";


PubSubClient client(wifiClient);

#define vitesseBaudRate_topic "Vitesse/baud/rate"
#define vitesseSerial_topic "Vitesse/serial"

#define debitCanVtotale "Debit/CanVehicule/totale"
#define PGNCanVTotale "Lecture/PGN/vehicule/totale"

#define debitCanVAs "Debit/CanVehicule/as"
#define asCanV "Lecture/PGN/vehicule/as"
#define identificationControleurSource "Identification/controleur/source"
#define lectureDataASVehicule "Lecture/data/as/vehicule"

#define debitCanVPGN            "Debit/CanVehicule/PGN"
#define PGNCanV_PGN             "Lecture/PGN/vehicule/PGN"

#define debitCanETotale            "Debit/CanEquipement/totale"
#define PGNCanETotale              "Lecture/PGN/equipement/totale"

#define debitCanEPGN            "Debit/CanEquipement/PGN"
#define PGNCanE_PGN             "Lecture/PGN/equipement/PGN"


char message_buff[100];
long lastMsg = 0;   
// long lastRecu = 0;
bool debugage = false;  
char msg[50];
int value = 0;
String consigne;


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arriver [");
  Serial.print(topic);
  Serial.print("] ");

  // Affichage sans écrire hors-borne
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // ➜ Copie sûre vers un tampon local
  static char rxBuf[128];
  const size_t maxCopy = sizeof(rxBuf) - 1;
  size_t n = (length < maxCopy) ? length : maxCopy;
  memcpy(rxBuf, payload, n);
  rxBuf[n] = '\0';

  consigne = String(rxBuf);
}

// Tentative de reconnexion MQTT NON-BLOQUANTE, avec anti-spam (intervalConnexionMQTT)
void reconnect() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (client.connected()) return;

  if (mqtt_server[0] == '\0') return;

  // Anti-spam 10 s
  unsigned long now = millis();
  if (now - dernierTempsConexionMQTT < intervalConnexionMQTT) return;
  dernierTempsConexionMQTT = now;

  Serial.println("Tentative de connexion au serveur MQTT...");
  bool ok = client.connect("Analyseur BUSCAN", mqttUser, mqttPassword);
  if (ok) {
    Serial.println("Connexion reussie a l'analyseur BUSCAN !");
    debugage = true;
    // subscribeTopics();
  } else {
    Serial.print("Erreur de connexion au serveur MQTT. Code: ");
    Serial.println(client.state());
    debugage = false;
  }
}

 static void mqtt_publish_speeds() {
  if (!client.connected()) return;

  char buf[24];

  // vitesseBaudRate -> "Vitesse/baud/rate"
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)vitesseBaudRate);
  client.publish(vitesseBaudRate_topic, buf /*, true  <- mets true si tu veux retenu */);

  // vitesseSerial -> "Vitesse/serial"
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)vitesseSerial);
  client.publish(vitesseSerial_topic, buf /*, true */);
 }
 
 static inline void mqtt_publish_pgn_bytes_decimal(const char* topic, const uint8_t b[8]) { // Publie 8 octets en décimal "000.000.000.000.000.000.000.000"
  if (!client.connected()) return;
  char s[64];
  snprintf(s, sizeof(s),
           "%03u.%03u.%03u.%03u.%03u.%03u.%03u.%03u",
           b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
  client.publish(topic, s /*, true  <- mets true si tu veux retenu */);
 }

static void ensureVersionBadgeOnTop() {
  if (!g_versionBadge) {
    g_versionBadge = lv_label_create(lv_layer_top());
    static char buf[32];
    snprintf(buf, sizeof(buf), "Version %s", APP_VERSION);
    lv_label_set_text(g_versionBadge, buf);

    // Style discret
    lv_obj_set_style_text_font(g_versionBadge, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(g_versionBadge, lv_color_white(), 0);
    lv_obj_set_style_bg_color(g_versionBadge, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_versionBadge, LV_OPA_60, 0);
    lv_obj_set_style_radius(g_versionBadge, 6, 0);
    lv_obj_set_style_pad_all(g_versionBadge, 6, 0);

    // Ne pas capter les clics ni être impacté par le layout
    lv_obj_add_flag(g_versionBadge, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_add_flag(g_versionBadge, LV_OBJ_FLAG_ADV_HITTEST);
  }
  // Collé en bas à droite
  lv_obj_align(g_versionBadge, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
}

//#####################################################################################

//############################ Capture BUSCAN Véhicule totale compteur + trames débit max ###################################

 // Estimation de débit très légère (fenêtre glissante ~100 ms)

 static volatile uint32_t vehRateCached = 0; // en trame/s (pour MQTT)
// --- Prototypes de timers HUD (pour éviter les erreurs "not declared")
static void vehHUDTimer_cb(lv_timer_t *t);
static void eqHUDTimer_cb(lv_timer_t *t);      // ← AJOUT
static void asHUDTimer_cb(lv_timer_t *t);      // ← (optionnel mais conseillé)
static void pgnHUDTimer_cb(lv_timer_t *t);     // ← (optionnel)
static void eqPgnHUDTimer_cb(lv_timer_t *t);   // ← (optionnel)



 // --- Conversion ultra-légère uint32_t -> ASCII base 10 (pas de printf, pas de String)
 static inline void u32toa10(uint32_t v, char *out) {
  char tmp[10];
  int i = 0;
  do { tmp[i++] = char('0' + (v % 10)); v /= 10; } while (v);
  int j = 0;
  while (i--) out[j++] = tmp[i];
  out[j] = '\0';
 }

 // --- Publish MQTT (ASCII) sans ultoa/printf
 static inline void mqtt_publish_u32 (const char *topic, uint32_t v) {
  if (!client.connected()) return;
  char buf[12];
  u32toa10(v, buf);
  client.publish(topic, buf);
 }


 // --- HUD lecture Véhicule Totale (LVGL)
 static lv_obj_t *dotVeh      = nullptr;   // témoin (petit rond qui clignote)
 static lv_timer_t *vehHUDTimer = nullptr;
 static lv_obj_t *vehCntVal = nullptr, *vehRateVal = nullptr, *vehLastVal = nullptr;

 // --- Stats instantanées
 static volatile uint32_t vehFrameCount = 0;   // (déjà déclaré ? garde UNE seule définition)
 static uint32_t vehLastPGN = 0xFFFFFFFF;
 static uint32_t vehFramesSinceTick = 0;
 static uint32_t vehLastTickMs = 0;
 static bool vehBlinkFlip = false;

 static volatile bool capVehiculeTotaleActive = false;
 static char serLine[64]; // Option performance: buffer de ligne (évite String)

 // Crée une ligne "clé : valeur" avec valeur en bleu ciel, renvoie le label valeur via outVal
 static void makeKVRow(lv_obj_t* parent, const char* key, lv_obj_t** outVal) {
  const lv_color_t colKey   = lv_color_white();
  const lv_color_t colValue = lv_color_hex(0x00BFFF); // bleu ciel

  lv_obj_t* row = lv_obj_create(parent);
  lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row, 0, 0);
  lv_obj_set_style_pad_all(row, 0, 0);
  lv_obj_set_layout(row, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 8, 0);

  // clé
  lv_obj_t* k = lv_label_create(row);
  lv_label_set_text(k, key);
  lv_obj_set_style_text_color(k, colKey, 0);
  lv_obj_set_style_text_font(k, &lv_font_montserrat_32, 0);

  // valeur (bleue)
  lv_obj_t* v = lv_label_create(row);
  lv_label_set_text(v, "--");
  lv_obj_set_style_text_color(v, colValue, 0);
  lv_obj_set_style_text_font(v, &lv_font_montserrat_32, 0);

  if (outVal) *outVal = v;
 }


 // PGN J1939 (29 bits). Si 11 bits, renvoie 0xFFFFFFFF.
 static inline uint32_t extrairePGN_fromID (const uint32_t id) {
  const uint8_t pf = (id >> 16) & 0xFF;
  const uint8_t ps = (id >>  8) & 0xFF;
  return (pf < 240) ? (uint32_t)(pf << 8) : (uint32_t)((pf << 8) + ps);
 }
 static inline uint32_t extrairePGN (const CANFDMessage &m) {
  return m.ext ? extrairePGN_fromID(m.id) : 0xFFFFFFFF;
 }

 // --- capture “Véhicule Totale”
 static inline void pollVehiculeTotale() {
  if (!capVehiculeTotaleActive) return;

  CANFDMessage m;

  // Caches locaux (moins d'accès aux globals = un poil plus rapide)
  uint32_t localCount  = vehFrameCount;
  uint32_t lastPGNSeen = vehLastPGN;
  bool     gotFrame    = false;

  // Fenêtre courte pour estimer le débit sans bloquer (10 Hz env.)
  static uint32_t rateFramesWin = 0;
  static uint32_t rateLastMs    = 0;
  static uint32_t rateCached    = 0;

  // --- Vider la FIFO *avant* de publier, pour ne pas rater de trames
  while (fdcan1.receiveFD0(m)) {
    gotFrame = true;
    localCount++;
    vehFramesSinceTick++;  // pour le HUD
    rateFramesWin++;       // pour le débit MQTT

    const uint32_t pgn = extrairePGN(m);

    if (pgn != 0xFFFFFFFF) {
      lastPGNSeen = pgn;
      // Log série très court (tu l’avais déjà)
      const int n = snprintf(serLine, sizeof(serLine),
                             "#%lu PGN=%lu\r\n",
                             (unsigned long)localCount,
                             (unsigned long)pgn);
      Serial.write((uint8_t*)serLine, (size_t)n);
    } else {
      const int n = snprintf(serLine, sizeof(serLine),
                             "#%lu PGN=N/A\r\n",
                             (unsigned long)localCount);
      Serial.write((uint8_t*)serLine, (size_t)n);
    }
  }

  if (!gotFrame) return; // rien reçu -> rien à publier

  // Commit des valeurs agrégées (une seule fois)
  vehFrameCount = localCount;
  vehLastPGN    = lastPGNSeen;

  // Mise à jour du débit (trames/s) toutes ~100 ms
  const uint32_t now = millis();
  const uint32_t dt  = now - rateLastMs;
  if (dt >= 100) {
    rateCached    = (rateFramesWin * 1000UL) / dt;
    rateFramesWin = 0;
    rateLastMs    = now;
  }

  // --- Publications MQTT (ASCII) : le plus vite possible après la rafle
  if (client.connected()) {
    // dernier PGN (0 si N/A)
    mqtt_publish_u32(PGNCanVTotale, (vehLastPGN != 0xFFFFFFFF) ? vehLastPGN : 0);
    // débit estimé en trames/s
    mqtt_publish_u32(debitCanVtotale, rateCached);
  }
 }


//###########################################################################################################################

//############################ Capture BUSCAN Equipement totale compteur + trames débit max ###################################
 
 
 static volatile bool capEquipementTotaleActive = false; // --- Capture "Équipement Totale"
 static volatile uint32_t eqFrameCount = 0; // Stats instantanées Équipement
 static uint32_t eqLastPGN = 0xFFFFFFFF;
 static uint32_t eqFramesSinceTick = 0;
 static uint32_t eqLastTickMs = 0;
 static bool     eqBlinkFlip = false;
 static char     serEqLine[64];
 static lv_obj_t   *dotEq = nullptr; // HUD LVGL Équipement
 static lv_timer_t *eqHUDTimer = nullptr;
 static lv_obj_t *eqCntVal  = nullptr,  *eqRateVal  = nullptr,  *eqLastVal  = nullptr;
 
 static inline void pollEquipementTotale() {
  if (!capEquipementTotaleActive) return;

  CANFDMessage m;

  // Fenêtre courte pour estimer le débit (publie ~toutes les 100 ms)
  static uint32_t rateFramesWin = 0;
  static uint32_t rateLastMs    = 0;
  static uint32_t rateCached    = 0;

  bool gotFrame = false;

  while (fdcan1.receiveFD0(m)) {
    gotFrame = true;
    eqFrameCount++;
    eqFramesSinceTick++;
    rateFramesWin++;

    const uint32_t pgn = extrairePGN(m);
    if (pgn != 0xFFFFFFFF) eqLastPGN = pgn;

    // Sortie série compacte
    if (pgn != 0xFFFFFFFF) {
      const int n = snprintf(serEqLine, sizeof(serEqLine),
                             "#%lu PGN=%lu\r\n",
                             (unsigned long)eqFrameCount, (unsigned long)pgn);
      Serial.write((uint8_t*)serEqLine, (size_t)n);
    } else {
      const int n = snprintf(serEqLine, sizeof(serEqLine),
                             "#%lu PGN=N/A\r\n",
                             (unsigned long)eqFrameCount);
      Serial.write((uint8_t*)serEqLine, (size_t)n);
    }
  }

  if (!gotFrame) return;

  // Mise à jour du débit (trames/s) toutes ~100 ms
  const uint32_t now = millis();
  const uint32_t dt  = now - rateLastMs;
  if (dt >= 100) {
    rateCached    = (rateFramesWin * 1000UL) / dt;
    rateFramesWin = 0;
    rateLastMs    = now;
  }

  // --- Publications MQTT (ASCII) comme pour les autres modes
  if (client.connected()) {

    // eqRateVal → débit estimé (trames/s)
    mqtt_publish_u32(debitCanETotale, rateCached);

    // eqLastVal → dernier PGN vu (0 si N/A)
    mqtt_publish_u32(PGNCanETotale, (eqLastPGN != 0xFFFFFFFF) ? eqLastPGN : 0);
  }
}


//###########################################################################################################################

//############################ Capture BUSCAN Véhicule par adresse source compteur + trames débit max ###################################
 
 // --------- Capture "Véhicule par Adresse Source" ---------
 static volatile bool capVehiculeASActive = false;

 static volatile uint32_t asFrameCount = 0;
 static uint32_t asLastPGN = 0xFFFFFFFF;
 static uint32_t asFramesSinceTick = 0;
 static uint32_t asLastTickMs = 0;
 static bool     asBlinkFlip = false;
 static char     serASLine[64];
 static uint8_t  currentSAFilter = 0xFF;    // SA sélectionné (0xFF = invalide)
 static lv_obj_t *asCntVal  = nullptr,  *asRateVal  = nullptr,  *asLastVal  = nullptr;

 // HUD LVGL (écran SCR_VEHICULE_LECT_AS)
 static lv_obj_t   *dotAS = nullptr;
 static lv_timer_t *asHUDTimer = nullptr;
 
 // Renvoie SA (bits 7..0) si trame étendue, sinon 0xFF
 static inline uint8_t extraireSA(const CANFDMessage &m) {
  return m.ext ? (uint8_t)(m.id & 0xFF) : 0xFF;
 }
 
 static inline void pollVehiculeAS() {
  if (!capVehiculeASActive) return;

  CANFDMessage m;

  // Fenêtre courte pour estimer le débit (publie ~toutes les 100 ms)
  static uint32_t rateFramesWin = 0;
  static uint32_t rateLastMs    = 0;
  static uint32_t rateCached    = 0;

  bool gotFrame = false;

  while (fdcan1.receiveFD0(m)) {
    if (!m.ext) continue;                            // pas J1939
    if (extraireSA(m) != currentSAFilter) continue;  // filtre SA

    gotFrame = true;
    asFrameCount++;
    asFramesSinceTick++;      // pour le HUD
    rateFramesWin++;          // pour le débit MQTT

    const uint32_t pgn = extrairePGN(m);
    if (pgn != 0xFFFFFFFF) {
      asLastPGN = pgn;
     
  if (client.connected()) { // Publier les 8 octets en décimal "000.000.000.000.000.000.000.000"
    uint8_t b[8] = {0};
    const uint8_t n = (m.len <= 8) ? m.len : 8;
    for (uint8_t i = 0; i < n; i++) b[i] = m.data[i];
    mqtt_publish_pgn_bytes_decimal(lectureDataASVehicule, b);
  }

      // Sortie série compacte
      const int n = snprintf(serASLine, sizeof(serASLine),
                             "#%lu SA=0x%02X PGN=%lu\r\n",
                             (unsigned long)asFrameCount,
                             (unsigned)currentSAFilter,
                             (unsigned long)pgn);
      Serial.write((uint8_t*)serASLine, (size_t)n);
    } else {
      const int n = snprintf(serASLine, sizeof(serASLine),
                             "#%lu SA=0x%02X PGN=N/A\r\n",
                             (unsigned long)asFrameCount,
                             (unsigned)currentSAFilter);
      Serial.write((uint8_t*)serASLine, (size_t)n);
    }
  }

  if (!gotFrame) return;

  // MAJ du débit (trames/s) toutes ~100 ms
  const uint32_t now = millis();
  const uint32_t dt  = now - rateLastMs;
  if (dt >= 100) {
    rateCached    = (rateFramesWin * 1000UL) / dt;
    rateFramesWin = 0;
    rateLastMs    = now;
  }

  // --- Publications MQTT (ASCII) comme pour "Véhicule totale"
  if (client.connected()) {

    // Dernier PGN vu (0 si N/A)
    mqtt_publish_u32(asCanV, (asLastPGN != 0xFFFFFFFF) ? asLastPGN : 0);

    // Débit estimé (trames/s)
    mqtt_publish_u32(debitCanVAs, rateCached);
  }
 }

 
 static void asHUDTimer_cb(lv_timer_t *t) {
  const uint32_t now = millis();
  const uint32_t dt  = now - asLastTickMs;
  if (dt == 0) return;

  const uint32_t rate = (asFramesSinceTick * 1000UL) / dt;

  if (asCntVal)  { static char b[20]; snprintf(b, sizeof(b), "%lu",   (unsigned long)asFrameCount); lv_label_set_text(asCntVal,  b); }
  if (asRateVal) { static char b[20]; snprintf(b, sizeof(b), "%lu/s", (unsigned long)rate);        lv_label_set_text(asRateVal, b); }
  if (asLastVal) {
    static char b[24];
    if (asLastPGN != 0xFFFFFFFF) snprintf(b, sizeof(b), "%lu", (unsigned long)asLastPGN);
    else                         snprintf(b, sizeof(b), "N/A");
    lv_label_set_text(asLastVal, b);
  }

  if (dotAS) {
    if (asFramesSinceTick > 0) {
      asBlinkFlip = !asBlinkFlip;
      lv_obj_clear_flag(dotAS, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_bg_opa(dotAS, asBlinkFlip ? LV_OPA_COVER : LV_OPA_50, 0);
    } else {
      lv_obj_set_style_bg_opa(dotAS, LV_OPA_20, 0);
    }
  }

  asFramesSinceTick = 0;
  asLastTickMs = now;
  (void)t;
 }

//###########################################################################################################################

//############################ Capture BUSCAN Véhicule par PGN ####################################

 // ======== Lecture par PGN (Véhicule) ========
 static volatile bool capVehiculePGNActive = false; // flag capture en cours
 static int currentPGNVehicule = -1;                // existe déjà chez toi (garde-le global)
 static lv_obj_t *lblVehPGNBytes = nullptr;         // label à MAJ en temps réel (zone d’octets)
 static volatile uint32_t vehPGNFrameCount = 0;     // compteur (optionnel)
 // static char serPGNLine[96];                        // buffer sortie série compacte
 
 // ======== HUD Lecture par PGN (Véhicule) ========
 static lv_timer_t *pgnHUDTimer = nullptr;
 static lv_obj_t *pgnCntVal = nullptr, *pgnRateVal = nullptr, *dotPGN = nullptr;
 static uint32_t vehPGNFramesSinceTick = 0;
 static uint32_t vehPGNLastTickMs = 0;
 static bool     vehPGNBlinkFlip = false;

 // -------- Lecture par PGN (Véhicule) --------
 static inline void pollVehiculePGN() {
  if (!capVehiculePGNActive) return;
  if (currentPGNVehicule < 0) return;

  CANFDMessage m;

  // Fenêtre courte pour estimer le débit (publication ~toutes les 100 ms)
  static uint32_t rateFramesWin = 0;
  static uint32_t rateLastMs    = 0;
  static uint32_t rateCached    = 0;

  bool gotFrame = false;

  while (fdcan1.receiveFD0(m)) {
    if (!m.ext) continue;
    const uint32_t pgn = extrairePGN(m);
    if (pgn == 0xFFFFFFFF) continue;
    if ((int)pgn != currentPGNVehicule) continue;

    gotFrame = true;
    vehPGNFrameCount++;
    vehPGNFramesSinceTick++;   // pour le HUD
    rateFramesWin++;           // pour le débit MQTT

    // Prépare 8 octets (padding 0 si DLC < 8)
    uint8_t b[8] = {0};
    const uint8_t n = (m.len <= 8) ? m.len : 8;
    for (uint8_t i = 0; i < n; i++) b[i] = m.data[i];
   
    mqtt_publish_pgn_bytes_decimal("Lecture/data/PGN/vehicule", b); // Publier les 8 octets au format décimal
    
    if (lblVehPGNBytes && lv_obj_is_valid(lblVehPGNBytes)) { // MAJ zone des octets (label global)
      char buf[64];
      snprintf(buf, sizeof(buf),
               "%03u.%03u.%03u.%03u.%03u.%03u.%03u.%03u",
               b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
      lv_label_set_text(lblVehPGNBytes, buf);
    }

    // Log série compact
    char serPGNLine[96];
    const int nser = snprintf(serPGNLine, sizeof(serPGNLine),
      "#%lu PGN=%lu DATA=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
      (unsigned long)vehPGNFrameCount, (unsigned long)pgn,
      b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
    Serial.write((uint8_t*)serPGNLine, (size_t)nser);
  }

  if (!gotFrame) return;

  // Mise à jour du débit (trames/s) toutes ~100 ms
  const uint32_t now = millis();
  const uint32_t dt  = now - rateLastMs;
  if (dt >= 100) {
    rateCached    = (rateFramesWin * 1000UL) / dt;
    rateFramesWin = 0;
    rateLastMs    = now;
  }

  // --- Publications MQTT (ASCII)
  if (client.connected()) {

    // PGN étudié (constante côté "lecture PGN") — on publie la valeur en cours
    mqtt_publish_u32(PGNCanV_PGN, (currentPGNVehicule >= 0) ? (uint32_t)currentPGNVehicule : 0);

    // Débit estimé (trames/s)
    mqtt_publish_u32(debitCanVPGN, rateCached);
  }
 }

 static void pgnHUDTimer_cb(lv_timer_t *t) {
  const uint32_t now = millis();
  const uint32_t dt  = now - vehPGNLastTickMs;
  if (dt == 0) return;

  const uint32_t rate = (vehPGNFramesSinceTick * 1000UL) / dt;

  if (pgnCntVal)  { static char b[20]; snprintf(b, sizeof(b), "%lu",   (unsigned long)vehPGNFrameCount); lv_label_set_text(pgnCntVal,  b); }
  if (pgnRateVal) { static char b[20]; snprintf(b, sizeof(b), "%lu/s", (unsigned long)rate);             lv_label_set_text(pgnRateVal, b); }

  if (dotPGN) {
    if (vehPGNFramesSinceTick > 0) {
      vehPGNBlinkFlip = !vehPGNBlinkFlip;
      lv_obj_clear_flag(dotPGN, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_bg_opa(dotPGN, vehPGNBlinkFlip ? LV_OPA_COVER : LV_OPA_50, 0);
    } else {
      lv_obj_set_style_bg_opa(dotPGN, LV_OPA_20, 0);
    }
  }

  vehPGNFramesSinceTick = 0;
  vehPGNLastTickMs = now;
  (void)t;
 }

//#################################################################################################

//############################ Capture BUSCAN Equipement par PGN ####################################

 extern int currentPGNEquipement; // défini plus bas avec tes globals UI
 // ======== Lecture par PGN (Équipement) ========
 static volatile bool capEquipementPGNActive = false;
 static lv_obj_t *lblEqPGNBytes = nullptr;          // label 8 octets (zone blanche)
 static lv_timer_t *eqPgnHUDTimer = nullptr;        // timer HUD
 static lv_obj_t *eqPgnCntVal = nullptr, *eqPgnRateVal = nullptr, *dotEqPGN = nullptr;

 static volatile uint32_t eqPGNFrameCount = 0;
 static uint32_t eqPGNFramesSinceTick = 0;
 static uint32_t eqPGNLastTickMs = 0;
 static bool     eqPGNBlinkFlip = false;

 // -------- Lecture par PGN (Équipement) --------
 static inline void pollEquipementPGN() {
  if (!capEquipementPGNActive) return;
  if (currentPGNEquipement < 0) return;

  CANFDMessage m;

  // Fenêtre courte pour estimer le débit (publication ~toutes les 100 ms)
  static uint32_t rateFramesWin = 0;
  static uint32_t rateLastMs    = 0;
  static uint32_t rateCached    = 0;

  bool gotFrame = false;

  while (fdcan1.receiveFD0(m)) {
    if (!m.ext) continue; // J1939 => étendu uniquement
    const uint32_t pgn = extrairePGN(m);
    if (pgn == 0xFFFFFFFF) continue;
    if ((int)pgn != currentPGNEquipement) continue;

    gotFrame = true;
    eqPGNFrameCount++;
    eqPGNFramesSinceTick++;
    rateFramesWin++;

    // 8 octets padding
    uint8_t b[8] = {0};
    const uint8_t n = (m.len <= 8) ? m.len : 8;
    for (uint8_t i = 0; i < n; i++) b[i] = m.data[i];
    
    mqtt_publish_pgn_bytes_decimal("Lecture/data/PGN/equipement", b); // Publier les 8 octets au format décimal

    // Zone 8 octets (décimal 000.000...)
    if (lblEqPGNBytes && lv_obj_is_valid(lblEqPGNBytes)) {
      char buf[64];
      snprintf(buf, sizeof(buf),
               "%03u.%03u.%03u.%03u.%03u.%03u.%03u.%03u",
               b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
      lv_label_set_text(lblEqPGNBytes, buf);
    }

    // Log série compact
    char line[96];
    const int nser = snprintf(line, sizeof(line),
      "[EQ]#%lu PGN=%lu DATA=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
      (unsigned long)eqPGNFrameCount, (unsigned long)pgn,
      b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
    Serial.write((uint8_t*)line, (size_t)nser);
  }

  if (!gotFrame) return;

  // Mise à jour du débit (trames/s) toutes ~100 ms
  const uint32_t now = millis();
  const uint32_t dt  = now - rateLastMs;
  if (dt >= 100) {
    rateCached    = (rateFramesWin * 1000UL) / dt;
    rateFramesWin = 0;
    rateLastMs    = now;
  }

  // --- Publications MQTT (ASCII), comme pour véhicule/AS
  if (client.connected()) {

    // PGN étudié (valeur courante)
    mqtt_publish_u32(PGNCanE_PGN, (currentPGNEquipement >= 0) ? (uint32_t)currentPGNEquipement : 0);

    // Débit estimé (trames/s)
    mqtt_publish_u32(debitCanEPGN, rateCached);
  }
 }

 static void eqPgnHUDTimer_cb(lv_timer_t *t) {
  const uint32_t now = millis();
  const uint32_t dt  = now - eqPGNLastTickMs;
  if (dt == 0) return;

  const uint32_t rate = (eqPGNFramesSinceTick * 1000UL) / dt;

  if (eqPgnCntVal)  { static char b[20]; snprintf(b, sizeof(b), "%lu",   (unsigned long)eqPGNFrameCount); lv_label_set_text(eqPgnCntVal,  b); }
  if (eqPgnRateVal) { static char b[20]; snprintf(b, sizeof(b), "%lu/s", (unsigned long)rate);            lv_label_set_text(eqPgnRateVal, b); }

  if (dotEqPGN) {
    if (eqPGNFramesSinceTick > 0) {
      eqPGNBlinkFlip = !eqPGNBlinkFlip;
      lv_obj_clear_flag(dotEqPGN, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_bg_opa(dotEqPGN, eqPGNBlinkFlip ? LV_OPA_COVER : LV_OPA_50, 0);
    } else {
      lv_obj_set_style_bg_opa(dotEqPGN, LV_OPA_20, 0);
    }
  }

  eqPGNFramesSinceTick = 0;
  eqPGNLastTickMs = now;
  (void)t;
 }

//#####################################################################################

// Déclaration globale des pages et images
 bool accueilActive = true;  // true = affichage page accueil
 lv_obj_t * screenPageAccueil;
 lv_obj_t * screenPageSelectionReseauCAN;
 lv_obj_t * screenVehicule;
 lv_obj_t * screenVehiculeTotale;
 lv_obj_t * screenVehiculeLectureTotale;
 lv_obj_t * screenVehiculeSelectionAdresseSource;
 lv_obj_t * screenCreateVehiculeLectureAdresseSource;
 lv_obj_t * screenVehiculePGN;
 lv_obj_t * screenVehiculeLecturePGN;
 lv_obj_t * screenEquipement;
 lv_obj_t * screenEquipementSelectionTotale;  
 lv_obj_t * screenEquipementLectureTotale;
 lv_obj_t * screenEquipementSelectionPGN;
 lv_obj_t * screenEquipementLecturePGN;
 lv_obj_t * screenConfigurationWifi;
// lv_obj_t * btnRetour;
 uint32_t start = 0;

 LV_IMG_DECLARE(wifiFort);
 LV_IMG_DECLARE(wifiBon);
 LV_IMG_DECLARE(wifiMoyen);
 LV_IMG_DECLARE(wifiFaible);
 LV_IMG_DECLARE(Wifi);
 LV_IMG_DECLARE(ArduinoPetit);
 LV_IMG_DECLARE(MSE);      
 LV_IMG_DECLARE(BUSCAN); 
 LV_IMG_DECLARE(Lancement);
 LV_IMG_DECLARE(relaisUn);
 LV_IMG_DECLARE(relaisDeux);
 LV_IMG_DECLARE(MQTT);
 LV_IMG_DECLARE(MosquittoLogo);

 Arduino_H7_Video          Display(800, 480, GigaDisplayShield);  // _____ Déclarations globales _______
 Arduino_GigaDisplayTouch  TouchDetector;


//############################## Page et callback des pages #############################################

 // SCREEN MANAGER — Lazy loading LVGL  

 // --- PROTOTYPES des createXXX utilisés dans getScreen() ---
 void createPageAccueil();
 void createPageSelectionReseauCAN();
 void createPageVehicule();
 void createVehiculeTotale();
 void createVehiculeLectureTotale();
 void createVehiculeAdresseSource();
 void createVehiculeLectureAdresseSource();
 void createVehiculePGN();
 void createVehiculeLecturePGN();
 void createPageEquipement();
 void createEquipementSelectionTotale();
 void createEquipementLectureTotale();
 void createEquipementSelectionPGN();
 void createEquipementLecturePGN();
 void createConfigurationWifi();
 void connecterWifiSTA();
 void updateWifiSignalIconByRSSI();
 static void purgeTopLayerTransient(); // prototype de la fonction
 void createConfigurationMqtt();

 extern lv_obj_t *kbWifi; // EXTERN des overlays utilisés par closeTransientOverlays()
 extern lv_obj_t *kbPGN;
 extern lv_obj_t *kbEquipementPGN;
 extern lv_obj_t *popupSource;

 //  Déclarations globales pour Véhicule PGN 
 lv_obj_t * labelPGNTitle = NULL;   // Titre en bas à droite
 lv_obj_t* kbPGN = nullptr;       // Clavier numérique
 lv_obj_t* taPGN = nullptr;       // Zone de texte pour saisir le PGN

 //  Déclarations globales pour Equipement PGN 
 lv_obj_t * labelEquipementPGNTitle = NULL;   // Label titre en bas à droite
 lv_obj_t * taEquipementPGN = NULL;           // Zone de texte pour saisir le PGN
 lv_obj_t * kbEquipementPGN = NULL;           // Clavier numérique
 int currentPGNEquipement = -1;   // -1 si aucun PGN saisi

 bool wifiPageActive = false;
 
 lv_obj_t *imgSignalWifi = nullptr; // Icône Wi-Fi en overlay global + mise à jour périodique 
 lv_timer_t *wifiTimer = nullptr;

 lv_obj_t *wifiPanel     = nullptr; // --- Panneau d'état Wi-Fi (objets LVGL)
 lv_obj_t *lblWifiIP     = nullptr;
 lv_obj_t *lblWifiEtat   = nullptr;

 lv_obj_t *taWifiSSID = nullptr;  // Champs de saisie + clavier pour la config Wi-Fi
 lv_obj_t *taWifiPass = nullptr;
 lv_obj_t *kbWifi     = nullptr;

 lv_obj_t *screenConfigurationMqtt = nullptr;
 lv_obj_t *btnMqttToggle = nullptr;

 lv_obj_t *mqttPanel   = nullptr;
 lv_obj_t *taMqttServer = nullptr;
 lv_obj_t *taMqttUser   = nullptr;
 lv_obj_t *taMqttPass   = nullptr;
 lv_obj_t *lblMqttEtat  = nullptr;
 lv_obj_t *lblMqttHost  = nullptr;
 bool mqttPageActive = false;

 lv_obj_t *imgMQTTPageConfig = nullptr;  // icône MQTT sur l’accueil
 lv_timer_t *mqttTimer = nullptr;
 lv_obj_t *imgMqttTop = nullptr;
 static volatile bool mqtt_connected_cached = false;

 static void buildMqttStatusPanel(lv_obj_t* parent);
 static void updateMqttStatusPanel();
 static void updateMqttConnectButtonUI();
 static void btnMqttConnect_cb(lv_event_t * e);
 static void btnMqttDisconnect_cb(lv_event_t * e);
 static void imgMQTTPageConfig_cb(lv_event_t * e);
 static void btnRetourInfoMqtt_cb(lv_event_t * e);
// static void mqtt_poll_cb(lv_timer_t *t);
 static void ta_focus_cb(lv_event_t * e);     // <— AJOUTER CETTE LIGNE
 static void kb_wifi_event_cb(lv_event_t * e); // (optionnel si tu l'utilises dans la page MQTT)
 
 static inline bool objAlive(lv_obj_t* o) {
  return o && lv_obj_is_valid(o);
 }

 static volatile int wifi_status_cached = WL_DISCONNECTED; // --- Cache d'état Wi-Fi (lu hors LVGL) ---
 static volatile int wifi_rssi_cached   = -127; // dBm
 static unsigned long last_wifi_poll_ms = 0;
  
 lv_obj_t *indicSquare = nullptr;  // --- Carré indicateur "E"/"V"
 lv_obj_t *indicLabel  = nullptr;
 static lv_style_t styleIndicSquare;
 static bool styleIndicSquare_init = false;

 static void destroyScreen(ScreenId id);
 static bool keepAlive(ScreenId id);
 static lv_obj_t* getScreen(ScreenId id);
 static void showScreen(ScreenId id);

 static lv_obj_t* g_currentScreen = nullptr; // État courant
 static ScreenId  g_currentId     = SCR_ACCUEIL;

 static void closeTransientOverlays() { // Helpers “overlays” (popups / claviers sur lv_layer_top)
  if (kbWifi)             { lv_obj_del(kbWifi);             kbWifi = nullptr; }
  if (kbPGN)              { lv_obj_del(kbPGN);              kbPGN = nullptr; }
  if (kbEquipementPGN)    { lv_obj_del(kbEquipementPGN);    kbEquipementPGN = nullptr; }
  if (popupSource)        { lv_obj_del(popupSource);        popupSource = nullptr; }
 }
 
 lv_obj_t* labelCalculateurInfo = nullptr;  // pour afficher SA, Nom, Boitier, Dénomination en global

 static void destroyScreen(ScreenId id) { // Détruire proprement un écran (et remettre son pointeur à nullptr)
  lv_obj_t** p = nullptr;
  switch (id) {
    case SCR_ACCUEIL:                   p = &screenPageAccueil; break;
    case SCR_SEL_CAN:                   p = &screenPageSelectionReseauCAN; indicSquare = nullptr; indicLabel  = nullptr; break; // Nettoyage des pointeurs quand on détruit l’écran
    case SCR_VEHICULE:                  p = &screenVehicule; break;
    case SCR_VEHICULE_TOTALE:           p = &screenVehiculeTotale; lblVehiculeBaud = nullptr; lblVehiculeSerial = nullptr; break;
    case SCR_VEHICULE_LECT_TOTALE:      p = &screenVehiculeLectureTotale; break;
    case SCR_VEHICULE_SEL_AS:           p = &screenVehiculeSelectionAdresseSource; lblVehiculeBaud = nullptr; lblVehiculeSerial = nullptr; break;
    case SCR_VEHICULE_LECT_AS:          p = &screenCreateVehiculeLectureAdresseSource; labelCalculateurInfo = nullptr; break; // important
    case SCR_VEHICULE_PGN:              p = &screenVehiculePGN; break;
    case SCR_VEHICULE_LECT_PGN:         p = &screenVehiculeLecturePGN; break;
    case SCR_EQUIPEMENT:                p = &screenEquipement; break;
    case SCR_EQUIPEMENT_SEL_TOTALE:     p = &screenEquipementSelectionTotale; lblEquipBaud = nullptr; lblEquipSerial = nullptr; break;
    case SCR_EQUIPEMENT_LECT_TOTALE:    p = &screenEquipementLectureTotale; break;
    case SCR_EQUIPEMENT_SEL_PGN:        p = &screenEquipementSelectionPGN; break;
    case SCR_EQUIPEMENT_LECT_PGN:       p = &screenEquipementLecturePGN; break;
    case SCR_WIFI:                      p = &screenConfigurationWifi; break;
    case SCR_MQTT:                      p = &screenConfigurationMqtt; break;
    default: break;
  }
  if (p && *p) { lv_obj_del(*p); *p = nullptr; }
 }
 
 static bool keepAlive(ScreenId id) {
  return (id == SCR_ACCUEIL) || (id == SCR_WIFI) || (id == SCR_MQTT); // <- AJOUT
 }

 static bool isReadingScreen(ScreenId id) {
  return id == SCR_VEHICULE_LECT_TOTALE
      || id == SCR_VEHICULE_LECT_AS
      || id == SCR_VEHICULE_LECT_PGN
      || id == SCR_EQUIPEMENT_LECT_TOTALE
      || id == SCR_EQUIPEMENT_LECT_PGN;
 }

 static inline lv_obj_t* ensureScreen(lv_obj_t*& screenPtr, void (*createFn)()) { // Petit helper pour “créer si nécessaire” via tes createXXX() existants
  if (!screenPtr) createFn();   // tes createXXX() remplissent screenPtr
  return screenPtr;
 }

 static lv_obj_t* getScreen(ScreenId id) { // Routeur : renvoie l’écran demandé (créé à la demande)
  switch (id) {
    case SCR_ACCUEIL:                return ensureScreen(screenPageAccueil, createPageAccueil);
    case SCR_SEL_CAN:                return ensureScreen(screenPageSelectionReseauCAN, createPageSelectionReseauCAN);
    case SCR_VEHICULE:               return ensureScreen(screenVehicule, createPageVehicule);
    case SCR_VEHICULE_TOTALE:        return ensureScreen(screenVehiculeTotale, createVehiculeTotale);
    case SCR_VEHICULE_LECT_TOTALE:   return ensureScreen(screenVehiculeLectureTotale, createVehiculeLectureTotale);
    case SCR_VEHICULE_SEL_AS:        return ensureScreen(screenVehiculeSelectionAdresseSource, createVehiculeAdresseSource);
    case SCR_VEHICULE_LECT_AS:       return ensureScreen(screenCreateVehiculeLectureAdresseSource, createVehiculeLectureAdresseSource);
    case SCR_VEHICULE_PGN:           return ensureScreen(screenVehiculePGN, createVehiculePGN);
    case SCR_VEHICULE_LECT_PGN:      return ensureScreen(screenVehiculeLecturePGN, createVehiculeLecturePGN);
    case SCR_EQUIPEMENT:             return ensureScreen(screenEquipement, createPageEquipement);
    case SCR_EQUIPEMENT_SEL_TOTALE:  return ensureScreen(screenEquipementSelectionTotale, createEquipementSelectionTotale);
    case SCR_EQUIPEMENT_LECT_TOTALE: return ensureScreen(screenEquipementLectureTotale, createEquipementLectureTotale);
    case SCR_EQUIPEMENT_SEL_PGN:     return ensureScreen(screenEquipementSelectionPGN, createEquipementSelectionPGN);
    case SCR_EQUIPEMENT_LECT_PGN:    return ensureScreen(screenEquipementLecturePGN, createEquipementLecturePGN);
    case SCR_WIFI:                   return ensureScreen(screenConfigurationWifi, createConfigurationWifi);
    case SCR_MQTT:                   return ensureScreen(screenConfigurationMqtt, createConfigurationMqtt);
    default:                         return ensureScreen(screenPageAccueil, createPageAccueil);
  }
 }

 static void showScreen(ScreenId id) {
  purgeTopLayerTransient();
  closeTransientOverlays();

  if (isReadingScreen(g_currentId) && !isReadingScreen(id)) {  // Si on QUITTE un écran de lecture, sécurité : on coupe tout
    stopAllCapturesAndTimers();
  }

  lv_obj_t* next = getScreen(id);
  if (!next) return;

  lv_scr_load(next);

  if (g_currentScreen && g_currentScreen != next && !keepAlive(g_currentId)) {
    destroyScreen(g_currentId);
  }
  g_currentScreen = next;
  g_currentId = id;
 }
  
 lv_obj_t *imgRelaisSelection = nullptr; // --- Indicateur visuel relais sur la page de sélection CAN
 BusProfile g_selectedBus = BusProfile::Equipement; // choix courant (par défaut Equipement)
 
 static void ensureBusIndicatorCreated() {
  if (!screenPageSelectionReseauCAN || !imgRelaisSelection) return;

  if (!styleIndicSquare_init) {
    lv_style_init(&styleIndicSquare);
    lv_style_set_bg_color(&styleIndicSquare, lv_color_hex(0x027D02)); // vert plein
    lv_style_set_bg_opa(&styleIndicSquare, LV_OPA_COVER);
    lv_style_set_border_width(&styleIndicSquare, 2);
    lv_style_set_border_color(&styleIndicSquare, lv_color_hex(0x0A2B0A));
    lv_style_set_radius(&styleIndicSquare, 10); // arrondi (pastille)    
    lv_style_set_pad_left(&styleIndicSquare, 12); // padding pour laisser respirer le texte
    lv_style_set_pad_right(&styleIndicSquare, 12);
    lv_style_set_pad_top(&styleIndicSquare, 6);
    lv_style_set_pad_bottom(&styleIndicSquare, 6);
    styleIndicSquare_init = true;
  }

  if (!indicSquare) {
    indicSquare = lv_obj_create(screenPageSelectionReseauCAN);    
    lv_obj_set_size(indicSquare, LV_SIZE_CONTENT, LV_SIZE_CONTENT); // largeur/hauteur auto selon le contenu (LVGL v8)
    lv_obj_add_style(indicSquare, &styleIndicSquare, 0);   
    lv_obj_align_to(indicSquare, imgRelaisSelection, LV_ALIGN_OUT_RIGHT_MID, 16, 0); // Aligner à droite de l'image, centré verticalement

    indicLabel = lv_label_create(indicSquare);
    lv_obj_set_style_text_color(indicLabel, lv_color_white(), 0);
    lv_obj_set_style_text_font(indicLabel, &lv_font_montserrat_32, 0);
    lv_label_set_long_mode(indicLabel, LV_LABEL_LONG_WRAP);
    lv_obj_center(indicLabel);
  } else {
    // si la pastille existe déjà, on s'assure qu'elle reste bien positionnée
    lv_obj_align_to(indicSquare, imgRelaisSelection, LV_ALIGN_OUT_RIGHT_MID, 16, 0);
  }
 }

 static void updateBusIndicator() {
  if (!screenPageSelectionReseauCAN || !imgRelaisSelection) return;
  ensureBusIndicatorCreated();

  if (indicLabel) {
    const char* txt = (g_selectedBus == BusProfile::Vehicule)
                      ? "CAN Vehicule"
                      : "CAN Equipement";
    lv_label_set_text(indicLabel, txt);
  }
 }

 static void updateRelaisImageOnSelectionPage() {
  if (screenPageSelectionReseauCAN && imgRelaisSelection) {
    if (g_selectedBus == BusProfile::Vehicule) {
      lv_img_set_src(imgRelaisSelection, &relaisUn);
    } else {
      lv_img_set_src(imgRelaisSelection, &relaisDeux);
    }
    lv_obj_clear_flag(imgRelaisSelection, LV_OBJ_FLAG_HIDDEN);
  }

  setRelayForCurrentSelection(); // synchronise le relais matériel
  updateBusIndicator();          // <-- AJOUT : met à jour le carré "E/V"
 }
 
 static void setRelayForCurrentSelection() {
  // relaisUn (Vehicule) => HIGH ; relaisDeux (Equipement) => LOW
  if (g_selectedBus == BusProfile::Vehicule) {
    digitalWrite(relaisSelectionBUSCAN, HIGH);
  } else {
    digitalWrite(relaisSelectionBUSCAN, LOW);
  }
 }

 static void btnSelVehicule_cb(lv_event_t * e); // callbacks spécifiques à la page de sélection CAN
 static void btnSelEquipement_cb(lv_event_t * e);

 static void btnSelVehicule_cb(lv_event_t * e) {
  (void)e;
  g_selectedBus = BusProfile::Vehicule;
  updateRelaisImageOnSelectionPage();
  applyBusProfile(g_selectedBus);     // <<<<<< init CAN + série 500k
  closePopupIfOpen();
  showScreen(SCR_VEHICULE);
 }

 static void btnSelEquipement_cb(lv_event_t * e) {
  (void)e;
  g_selectedBus = BusProfile::Equipement;
  updateRelaisImageOnSelectionPage();
  applyBusProfile(g_selectedBus);     // <<<<<< init CAN + série 250k
  closePopupIfOpen();
  showScreen(SCR_EQUIPEMENT);
 }

 static void imgMQTTPageConfig_cb(lv_event_t * e) {
  if (kbWifi) { lv_obj_del(kbWifi); kbWifi = nullptr; }
  closePopupIfOpen();
  Serial.println("Appui bouton MQTT -> page de configuration MQTT");
  showScreen(SCR_MQTT);
  mqttPageActive = true;
  accueilActive  = false;    // très important pour neutraliser l’auto-basculement
 }

 static void btnRetourInfoMqtt_cb(lv_event_t * e) {
  if (kbWifi) { lv_obj_del(kbWifi); kbWifi = nullptr; }
  closePopupIfOpen();
  showScreen(SCR_ACCUEIL);
  // >>> AJOUTS
  mqttPageActive = false;
  accueilActive  = true;
  start = millis();
 }

 static void buildMqttStatusPanel(lv_obj_t* parent) {
  if (mqttPanel) return; // déjà créé

  // ---- Panneau conteneur ----
  mqttPanel = lv_obj_create(parent);
  lv_obj_set_size(mqttPanel, 740, 330);
  lv_obj_align(mqttPanel, LV_ALIGN_TOP_MID, 0, 70);
  lv_obj_set_style_bg_color(mqttPanel, lv_color_hex(0x335E91), 0);
  lv_obj_set_style_bg_opa(mqttPanel, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(mqttPanel, 8, 0);
  lv_obj_set_style_pad_all(mqttPanel, 12, 0);
  lv_obj_set_style_border_width(mqttPanel, 1, 0);
  lv_obj_set_style_border_color(mqttPanel, lv_color_hex(0xA89E9E), 0);

  // Petite utilitaire pour les textareas
  auto setup_ta = [](lv_obj_t* ta, const char* placeholder, int w, int h) {
    lv_obj_set_size(ta, w, h);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_max_length(ta, 64);
    lv_textarea_set_placeholder_text(ta, placeholder);
    lv_obj_set_style_text_font(ta, &lv_font_montserrat_32, 0);
    // Texte noir
    lv_obj_set_style_text_color(ta, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ta, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_FOCUSED);
  };

  // ---------- Ligne 1 : Adresse du serveur ----------
  lv_obj_t *lblHostTitle = lv_label_create(mqttPanel);
  lv_label_set_text(lblHostTitle, "Adresse du serveur");
  lv_obj_set_style_text_font(lblHostTitle, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblHostTitle, lv_color_white(), 0);
  lv_obj_align(lblHostTitle, LV_ALIGN_TOP_LEFT, 10, 0);

  taMqttServer = lv_textarea_create(mqttPanel);
  setup_ta(taMqttServer, "IP serveur", 700, 48);
  lv_textarea_set_max_length(taMqttServer, sizeof(mqtt_server) - 1);
  lv_textarea_set_text(taMqttServer, "");
  lv_obj_add_event_cb(taMqttServer, ta_focus_cb, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(taMqttServer, ta_focus_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_align(taMqttServer, LV_ALIGN_TOP_LEFT, 4, 40);

  // ---------- Ligne 2 : Nom d'utilisateur (gauche) + Broker (droite) ----------
  lv_obj_t *lblUserTitle = lv_label_create(mqttPanel);
  lv_label_set_text(lblUserTitle, "Nom d'utilisateur");
  lv_obj_set_style_text_font(lblUserTitle, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblUserTitle, lv_color_white(), 0);
  lv_obj_align(lblUserTitle, LV_ALIGN_TOP_LEFT, 10, 105);

  // Label "Broker : ..." à DROITE de la ligne utilisateur
  lblMqttHost = lv_label_create(mqttPanel);
  lv_obj_set_style_text_font(lblMqttHost, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(lblMqttHost, lv_color_white(), 0);
  lv_obj_set_width(lblMqttHost, 360);                  // largeur dispo à droite
  lv_obj_set_style_text_align(lblMqttHost, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(lblMqttHost, LV_ALIGN_TOP_RIGHT, -10, 105); // même y que le titre utilisateur

  taMqttUser = lv_textarea_create(mqttPanel);
  setup_ta(taMqttUser, "UserName", 340, 58);          // un peu plus court pour laisser respirer la droite
  lv_textarea_set_max_length(taMqttUser, sizeof(mqttUser) - 1);
  lv_textarea_set_text(taMqttUser, "");
  lv_obj_add_event_cb(taMqttUser, ta_focus_cb, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(taMqttUser, ta_focus_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_align(taMqttUser, LV_ALIGN_TOP_LEFT, 4, 145);

  // ---------- Ligne 3 : Mot de passe tout en bas ----------
  lv_obj_t *lblPassTitle = lv_label_create(mqttPanel);
  lv_label_set_text(lblPassTitle, "Mot de passe");
  lv_obj_set_style_text_font(lblPassTitle, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblPassTitle, lv_color_white(), 0);
  lv_obj_align(lblPassTitle, LV_ALIGN_TOP_LEFT, 10, 205);

  taMqttPass = lv_textarea_create(mqttPanel);
  setup_ta(taMqttPass, "Password", 700, 48);
  lv_textarea_set_max_length(taMqttPass, sizeof(mqttPassword) - 1);
  lv_textarea_set_password_mode(taMqttPass, true);
  lv_textarea_set_text(taMqttPass, "");
  lv_obj_add_event_cb(taMqttPass, ta_focus_cb, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(taMqttPass, ta_focus_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_align(taMqttPass, LV_ALIGN_TOP_LEFT, 4, 245);

  // ---------- État (en bas à droite de la ligne mot de passe) ----------
  lblMqttEtat = lv_label_create(mqttPanel);
  lv_label_set_text(lblMqttEtat, "Deconnecter");
  lv_obj_set_style_text_font(lblMqttEtat, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblMqttEtat, lv_color_white(), 0);
  lv_obj_set_width(lblMqttEtat, 260);
  lv_obj_set_style_text_align(lblMqttEtat, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(lblMqttEtat, LV_ALIGN_TOP_RIGHT, -10, 205);

  // MAJ des textes "Broker" et "Etat"
  updateMqttStatusPanel();
 }

 static void updateMqttStatusPanel() { // --- MAJ des labels d'état/host MQTT ---
  if (!mqttPanel) return;

  static char bufHost[96];
  static char bufEtat[24];

  const bool connected = client.connected();
  const char* host = (mqtt_server[0] != '\0') ? mqtt_server : "(non defini)";

  snprintf(bufHost, sizeof(bufHost), "Broker : %s:1883", host);
  snprintf(bufEtat, sizeof(bufEtat), "%s", connected ? "Connection ok" : "Deconnecter");

  if (lblMqttHost) lv_label_set_text(lblMqttHost, bufHost);
  if (lblMqttEtat) lv_label_set_text(lblMqttEtat, bufEtat);
 }

 static void updateMqttConnectButtonUI() {
  if (!btnMqttToggle) return;
  initClickStyles();

  lv_obj_remove_style_all(btnMqttToggle);
  lv_obj_set_size(btnMqttToggle, 240, 60);
  lv_obj_align(btnMqttToggle, LV_ALIGN_BOTTOM_MID, 0, -10);

  lv_obj_t *lbl = lv_obj_get_child(btnMqttToggle, 0);

  // base verte (coins, bordure) comme Wi-Fi
  applyStyleBtnRetour(btnMqttToggle);
  // effet de clic commun
  lv_obj_add_style(btnMqttToggle, &styleBtnBaseClick, 0);
  lv_obj_add_flag(btnMqttToggle, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_text_color(btnMqttToggle, lv_color_white(), 0);

  // Clear anciens callbacks
  lv_obj_remove_event_cb(btnMqttToggle, btnMqttConnect_cb);
  lv_obj_remove_event_cb(btnMqttToggle, btnMqttDisconnect_cb);

  if (client.connected()) {
    // ROUGE "Déconnecter"
    applyStyleBtnDanger(btnMqttToggle);
    if (lbl) lv_label_set_text(lbl, "Deconnecter");
    lv_obj_add_event_cb(btnMqttToggle, btnMqttDisconnect_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(btnMqttToggle, &styleBtnPressedRed, LV_STATE_PRESSED);
  } else {
    // VERT "Se connecter"
    if (lbl) lv_label_set_text(lbl, "Se connecter");
    lv_obj_add_event_cb(btnMqttToggle, btnMqttConnect_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(btnMqttToggle, &styleBtnPressedGreen, LV_STATE_PRESSED);
  }
 }
 
 static void btnMqttConnect_cb(lv_event_t * e) {
  // Lire les champs
  const char* host = taMqttServer ? lv_textarea_get_text(taMqttServer) : "";
  const char* usr  = taMqttUser   ? lv_textarea_get_text(taMqttUser)   : "";
  const char* pwd  = taMqttPass   ? lv_textarea_get_text(taMqttPass)   : "";

  strncpy(mqtt_server, host, sizeof(mqtt_server)-1);
  mqtt_server[sizeof(mqtt_server)-1] = '\0';
  strncpy(mqttUser, usr, sizeof(mqttUser)-1);
  mqttUser[sizeof(mqttUser)-1] = '\0';
  strncpy(mqttPassword, pwd, sizeof(mqttPassword)-1);
  mqttPassword[sizeof(mqttPassword)-1] = '\0';

  // (Re)configurer l’hôte et tenter une connexion
  client.setServer(mqtt_server, 1883);
  client.setKeepAlive(30);     // 30–60 s : charge plus faible
  client.setSocketTimeout(3);  // 3 s : moins de micro timeouts

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[MQTT] Tentative de connexion immediate...");
    bool ok = client.connect("Analyseur BUSCAN", mqttUser, mqttPassword);
    if (ok) {
      Serial.println("[MQTT] Connecte !");
      debugage = true;
      mqtt_publish_speeds();
    } else {
      Serial.print("[MQTT] Echec, code: ");
      Serial.println(client.state());
      debugage = false;
    }
  } else {
    Serial.println("[MQTT] Wi-Fi non connecte: la reconnexion sera tentee plus tard.");
  }

  updateMqttStatusPanel();
  updateMqttConnectButtonUI();
  mqtt_connected_cached = client.connected();  // refresh immédiat
  setMqttTopIconVisible(mqtt_connected_cached && wifi_status_cached == WL_CONNECTED);
 }

 static void btnMqttDisconnect_cb(lv_event_t * e) {
  if (client.connected()) client.disconnect();
  Serial.println("[MQTT] Deconnecte.");
  updateMqttStatusPanel();
  updateMqttConnectButtonUI();
  mqtt_connected_cached = client.connected();  // devrait être false
  setMqttTopIconVisible(false);
 }

 static inline void mqtt_publish_str(const char* topic, const char* s) { // publication du boitier source sélectionné
  if (!client.connected()) return;
  client.publish(topic, s ? s : "");
 }
 //###################################################### Adresse Source calculateurs ########################################################

 lv_obj_t* popupSource = nullptr;
 lv_obj_t *titreDescription;
 lv_obj_t *imgLancement;
 lv_obj_t * btnChoisir;  // === Déclare btnChoisir en global ===
 
 struct Calculateur {
  uint8_t sa;
  const char* nom;
  const char* boitier;
  const char* denomination;
 };

 static const Calculateur calculateurs[] = { // Structuration du tableau de calculateur
  {0x00, "ECU", "Controleur du moteur", "-A021-"},
  {0x03, "PTA", "Controleur de transmission", "-A013-"},
  {0x06, "EIC", "Controleur du domaine de la transmission", "-A046-"},
  {0x13, "XBC", "Controleur de direction autotrac", "-A132-"},
  {0x19, "ETC", "Controleur de climatisation", "-A058-"},
  {0x1C, "IR7/8", "Controleur StarFire", "-A052-"},
  {0x1D, "ICD", "Controleur anti demarage", "-A133-"},
  {0x1E, "OST", "Controleur de domaine de la plateforme de conduite", "-A047-"},
  {0x22, "DHP", "Controleur des options et du circuit hydraulique", "-A045-"},
  {0x26, "VTV", "Processeur", "-A156-"},
  {0x28, "PDU", "Console d'angle", "-A043-"},
  {0x31, "OSB", "Controleur de domaine de la plateforme de conduite", "-A047-"},
  {0x32, "OSC", "Controleur de domaine de la plateforme de conduite", "-A047-"},
  {0x37, "OSO", "Controleur de la cabine", "-A048-"},
  {0x47, "DCT", "Controleur du domaine de la transmission", "-A046-"},
  {0x48, "DCP", "Controleur du domaine de la transmission", "-A046-"},
  {0x49, "DCS", "Controleur du domaine de la transmission", "-A046-"},
  {0x4C, "DE7", "Controleur de la radio", "-A140-"},
  {0x8B, "CSM", "Controleur de module de conatacteur de cabine", "-S175-"},
  {0x93, "DHB", "Controleur des options et du circuit hydraulique", "-A045-"},
  {0x98, "HV1", "Controleur d'attelage arrière", "-M045-"},
  {0x9C, "GR7/8", "Controleur StarFire", "-A052-"},
  {0xA1, "TM1", "Controleur de valve I", "-A036-"},
  {0xA2, "TM2", "Controleur de valve II", "-A037-"},
  {0xA3, "TM3", "Controleur de valve III", "-A038-"},
  {0xA4, "TM4", "Controleur de valve IV", "-A039-"},
  {0xAB, "TMK", "Controleur de valve XI", "-A033-"},
  {0xAC, "TML", "Controleur de valve XII", "-A034-"},
  {0xAD, "TMM", "Controleur de valve XIII", "-A035-"},
  {0xC8, "XBB", "Controleur de direction autotrac", "-A132-"},
  {0x96, "RLC", "Controleur du tableau electrique central du pavillon", "-A049-"},
  {0x95, "CLC", "Controleur du tableau electrique central de cabine", "-A050-"},
  {0xF0, "TEC", "Controleur du domaine des equipements du tracteur", "-A044-"},
  {0xFB, "JDL", "Controleur JDLink", "-A155-"}
 };

 const Calculateur* selectedCalculateur = nullptr; // Calculateur sélectionné actuellement

void popupSelect_cb(lv_event_t * e) {
  // Bouton cliqué (dans le popup)
  lv_obj_t* btn_popup = (lv_obj_t*) lv_event_get_target(e);
  if (!btn_popup) return;

  // Récupérer le texte affiché sur le bouton cliqué
  lv_obj_t* label = lv_obj_get_child(btn_popup, 0);
  const char* texte = label ? lv_label_get_text(label) : "";

  Serial.print("Source choisie : ");
  Serial.println(texte);

  // 1) Trouver le calculateur sélectionné
  selectedCalculateur = nullptr;
  for (size_t i = 0; i < sizeof(calculateurs)/sizeof(calculateurs[0]); i++) {
    if (strcmp(calculateurs[i].nom, texte) == 0) {
      selectedCalculateur = &calculateurs[i];
      break;
    }
  }

  // 2) Publier la dénomination UNE FOIS, après la recherche
  if (selectedCalculateur) {
    const char* denom = (selectedCalculateur->denomination) ? selectedCalculateur->denomination : "";
    mqtt_publish_str(identificationControleurSource, denom);
    Serial.print("[MQTT] Publication denomination: ");
    Serial.println(denom);
  } else {
    Serial.println("[WARN] Calculateur non trouvé, aucune publication MQTT.");
  }

  // 3) Mettre à jour le libellé du bouton principal (passé en user_data)
  lv_obj_t* main_btn  = (lv_obj_t*) lv_event_get_user_data(e);
  if (main_btn) {
    lv_obj_t* main_label = lv_obj_get_child(main_btn, 0);
    if (main_label) {
      lv_label_set_text(main_label, texte);
      lv_obj_update_layout(main_btn);
      lv_obj_invalidate(main_btn);
    }
  }

  // 4) Fermer le popup
  if (popupSource) {
    lv_obj_del(popupSource);
    popupSource = nullptr;
  }
}


 static void popup_bg_click_cb(lv_event_t * e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED && popupSource) {
    lv_obj_del(popupSource);
    popupSource = nullptr;
  }
 }

 static void popup_on_delete(lv_event_t * e) { // à placer près de popup_bg_click_cb
  (void)e; 
  popupSource = nullptr;
 }

 static inline void closePopupIfOpen() {
  if (popupSource) { lv_obj_del(popupSource); popupSource = nullptr; }
 }

 void openSourcePopup(lv_event_t * e) { // Remet le popup complet de sélection de source (sur lv_layer_top)

  Serial.println("openSourcePopup()");
  lv_obj_t* main_btn = (lv_obj_t*) lv_event_get_target(e);

  closePopupIfOpen(); // Fermer un éventuel ancien popup

  popupSource = lv_obj_create(lv_layer_top()); // ————— Overlay plein écran semi-transparent pour capter les clics hors-popup —————
  lv_obj_set_size(popupSource, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
  lv_obj_align(popupSource, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(popupSource, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(popupSource, LV_OPA_50, 0);   // voile
  lv_obj_clear_flag(popupSource, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(popupSource, LV_OBJ_FLAG_CLICKABLE);  // clic fond -> fermer
  lv_obj_add_event_cb(popupSource, popup_bg_click_cb, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(popupSource, popup_on_delete,   LV_EVENT_DELETE, NULL);

  lv_obj_t* card = lv_obj_create(popupSource);  // ————— La “carte” popup au centre (enfant de l’overlay) —————
  lv_obj_set_size(card, 760, 440);
  lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x417BE8), 0);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(card, 12, 0);
  lv_obj_set_style_border_width(card, 2, 0);
  lv_obj_set_style_border_color(card, lv_color_hex(0x333333), 0);
  lv_obj_set_style_pad_all(card, 12, 0);

  lv_obj_t* title = lv_label_create(card);   // ————— Titre —————
  lv_label_set_text(title, "Choisir une source logiciel");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

  lv_obj_t* cont = lv_obj_create(card); // ————— Conteneur scrollable pour les boutons —————
  const int cont_w = 720, cont_h = 360, gap = 8, cols = 6;
  lv_obj_set_size(cont, cont_w, cont_h);
  lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 56);
  lv_obj_set_style_bg_color(cont, lv_color_hex(0xFFFACD), 0);
  lv_obj_set_style_radius(cont, 6, 0);
  lv_obj_set_style_border_width(cont, 1, 0);
  lv_obj_set_style_border_color(cont, lv_color_hex(0xCCCC00), 0);
  
  lv_obj_set_layout(cont, LV_LAYOUT_FLEX); // Flex + scroll vertical
  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_column(cont, gap, 0);
  lv_obj_set_style_pad_row(cont, gap, 0);
  lv_obj_set_style_pad_all(cont, gap, 0);
  lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_AUTO);
  lv_obj_set_scroll_dir(cont, LV_DIR_VER);
 
  int btn_width  = (cont_w - gap * (cols - 1)) / cols; // Taille des boutons (répartition sur 'cols' colonnes)
  int btn_height = 56;

  size_t nb = sizeof(calculateurs) / sizeof(calculateurs[0]);  // ————— Boutons de source —————
  for (size_t i = 0; i < nb; i++) {
    lv_obj_t* btn = lv_btn_create(cont);
    lv_obj_set_size(btn, btn_width, btn_height);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, calculateurs[i].nom);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_32, 0);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn, popupSelect_cb, LV_EVENT_CLICKED, main_btn); // Passer le bouton principal 'main_btn' en user_data pour mettre à jour son libellé

  }
 
   lv_obj_t* btnClose = lv_btn_create(card); // ————— Bouton Fermer (en bas de la carte) —————
   lv_obj_set_size(btnClose, 140, 48);
   lv_obj_align(btnClose, LV_ALIGN_BOTTOM_MID, 0, -8);
   lv_obj_add_event_cb(btnClose, [](lv_event_t*){
    if (popupSource) { lv_obj_del(popupSource); popupSource = nullptr; }
    }, LV_EVENT_CLICKED, NULL);

   lv_obj_t* lblClose = lv_label_create(btnClose);
   lv_label_set_text(lblClose, "Fermer");
   lv_obj_center(lblClose);

  Serial.println("openSourcePopup(): popup prêt");
 }

 void updateCalculateurTitles(const char* nom, const char* boitier, const char* denomination) {
  static char buffer[256];
  snprintf(buffer, sizeof(buffer), "Description du logiciel :\n\nNom : %s\nBoitier : %s\nDénomination : %s", nom, boitier, denomination);
  lv_label_set_text(titreDescription, buffer);

 }

 void btn_event_cb(lv_event_t * e) { // Callback de détection d'appui des boutons
  Serial.println("Bouton cliqué !");
 } 

 lv_obj_t* creationTitre(lv_obj_t* parent, const char* texte, const lv_font_t* font, lv_align_t align, int x_offset, int y_offset) {
  lv_obj_t* label = lv_label_create(parent);
  lv_label_set_text(label, texte);
  lv_obj_set_style_text_font(label, font, 0);  // LVGL accepte const ici
  lv_obj_align(label, align, x_offset, y_offset);
  return label;

 }

 lv_obj_t* creationBouton(lv_obj_t* parent, const char* texte, int w, int h, lv_align_t align, int x_offset, int y_offset, lv_event_cb_t cb) {
  static lv_style_t btn_style;
  static bool btn_style_inited = false;
  if (!btn_style_inited) {
    lv_style_init(&btn_style);
    lv_style_set_text_font(&btn_style, &lv_font_montserrat_32);
    btn_style_inited = true;
  }

  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_align(btn, align, x_offset, y_offset);
  if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
 
  lv_obj_set_style_border_width(btn, 3, 0); // Style du bouton
  lv_obj_set_style_border_color(btn, lv_color_hex(0x212B63), 0);
  lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);

  lv_obj_t* label = lv_label_create(btn);  // Label
  lv_label_set_text(label, texte);
  lv_obj_center(label);
  lv_obj_add_style(label, &btn_style, 0);
  lv_obj_set_style_text_color(label, lv_color_white(), 0);
  return btn;
 }

 static lv_style_t styleBtnRetour; // Déclaration globale pour les boutons retour
 static bool styleBtnRetour_init = false;

 void applyStyleBtnRetour(lv_obj_t *btn) {
  if (!styleBtnRetour_init) {
    lv_style_init(&styleBtnRetour);
    lv_style_set_bg_color(&styleBtnRetour, lv_color_hex(0x027D02));
    lv_style_set_bg_opa(&styleBtnRetour, LV_OPA_COVER);   // <<< AJOUT
    lv_style_set_radius(&styleBtnRetour, 10);
    lv_style_set_border_color(&styleBtnRetour, lv_color_hex(0x0A2B0A));
    lv_style_set_border_width(&styleBtnRetour, 2);
    static lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, LV_STYLE_BORDER_WIDTH};
    static lv_style_transition_dsc_t trans;
    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 400, 0, NULL);
    lv_style_set_transition(&styleBtnRetour, &trans);
    styleBtnRetour_init = true;
   }
  lv_obj_add_style(btn, &styleBtnRetour, 0);
 }

 void Signature(lv_obj_t * parent) {  // Fonction pour ajouter signature + logo Arduino

  lv_obj_t * signature = lv_label_create(parent);
  lv_label_set_text(signature, "Ludovic COSSON");
  lv_obj_set_style_text_font(signature, &lv_font_montserrat_24, 0);
  lv_obj_align(signature, LV_ALIGN_BOTTOM_RIGHT, -180, -10);

  lv_obj_t * imgArduino = lv_img_create(parent); // Logo Arduino à gauche de la signature
  lv_img_set_src(imgArduino, &ArduinoPetit);
  lv_obj_align_to(imgArduino, signature, LV_ALIGN_OUT_LEFT_MID, 200, -40);

 }

 static void btnRetourAccueil_cb(lv_event_t * e) {
  closePopupIfOpen();
  showScreen(SCR_ACCUEIL);
  accueilActive = true;
  start = millis();
 }

 void btnVehicule_cb(lv_event_t * e) {
  closePopupIfOpen();
  Serial.println("Page Véhicule ouverte");
  showScreen(SCR_VEHICULE);
 }

 void btnRetourVehicule_cb(lv_event_t * e) { // Callback du bouton retour de la page createPageVehicule
  closePopupIfOpen();
  Serial.println("Retour à la page boutons depuis Véhicule");
  showScreen(SCR_SEL_CAN);
 }

 void btnVehiculeTotale_cb(lv_event_t * e) { // Callback pour ouvrir l'écran "Véhicule Totale"
  closePopupIfOpen();
  Serial.println("Page Véhicule Totale ouverte");
  showScreen(SCR_VEHICULE_TOTALE);  // charge l'écran correspondant
 }
 
 // === Définition du timer HUD "Véhicule totale" ===
static void vehHUDTimer_cb(lv_timer_t *t) {
  const uint32_t now = millis();
  const uint32_t dt  = now - vehLastTickMs;
  if (dt == 0) return;

  const uint32_t rate = (vehFramesSinceTick * 1000UL) / dt;

  if (vehCntVal)  { static char b[20]; snprintf(b, sizeof(b), "%lu",   (unsigned long)vehFrameCount); lv_label_set_text(vehCntVal,  b); }
  if (vehRateVal) { static char b[20]; snprintf(b, sizeof(b), "%lu/s", (unsigned long)rate);          lv_label_set_text(vehRateVal, b); }
  if (vehLastVal) {
    static char b[24];
    if (vehLastPGN != 0xFFFFFFFF) snprintf(b, sizeof(b), "%lu", (unsigned long)vehLastPGN);
    else                          snprintf(b, sizeof(b), "N/A");
    lv_label_set_text(vehLastVal, b);
  }

  if (dotVeh) {
    if (vehFramesSinceTick > 0) {
      vehBlinkFlip = !vehBlinkFlip;
      lv_obj_clear_flag(dotVeh, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_bg_opa(dotVeh, vehBlinkFlip ? LV_OPA_COVER : LV_OPA_50, 0);
    } else {
      lv_obj_set_style_bg_opa(dotVeh, LV_OPA_20, 0);
    }
  }

  vehFramesSinceTick = 0;
  vehLastTickMs = now;
  (void)t;
}

 // === Timer HUD "Equipement totale" ===
 static void eqHUDTimer_cb(lv_timer_t *t) {
  const uint32_t now = millis();
  const uint32_t dt  = now - eqLastTickMs;
  if (dt == 0) return;

  const uint32_t rate = (eqFramesSinceTick * 1000UL) / dt;

  if (eqCntVal)  { static char b[20]; snprintf(b, sizeof(b), "%lu",   (unsigned long)eqFrameCount); lv_label_set_text(eqCntVal,  b); }
  if (eqRateVal) { static char b[20]; snprintf(b, sizeof(b), "%lu/s", (unsigned long)rate);         lv_label_set_text(eqRateVal, b); }
  if (eqLastVal) {
    static char b[24];
    if (eqLastPGN != 0xFFFFFFFF) snprintf(b, sizeof(b), "%lu", (unsigned long)eqLastPGN);
    else                         snprintf(b, sizeof(b), "N/A");
    lv_label_set_text(eqLastVal, b);
  }

  if (dotEq) {
    if (eqFramesSinceTick > 0) {
      eqBlinkFlip = !eqBlinkFlip;
      lv_obj_clear_flag(dotEq, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_bg_opa(dotEq, eqBlinkFlip ? LV_OPA_COVER : LV_OPA_50, 0);
    } else {
      lv_obj_set_style_bg_opa(dotEq, LV_OPA_20, 0);
    }
  }

  eqFramesSinceTick = 0;
  eqLastTickMs = now;
  (void)t;
 }

 void btnRetourVehiculeLecture_cb(lv_event_t * e) { // --- Retour depuis l'écran de lecture totale

  (void)e;
  closePopupIfOpen();
 
  capVehiculeTotaleActive = false; // Stop capture + HUD
  if (vehHUDTimer) {
    lv_timer_del(vehHUDTimer);
    vehHUDTimer = nullptr;
  }
  Serial.print(F("Lecture totale Vehicule: ARRET, total="));
  Serial.println(vehFrameCount);
  
  showScreen(SCR_VEHICULE); // Retour à l'écran "Véhicule"
 }

 void imgLancement_cb(lv_event_t * e) { // --- Lancement lecture totale (depuis screenVehiculeTotale)

  (void)e;
  closePopupIfOpen();

  g_selectedBus = BusProfile::Vehicule; // Force le profil "Véhicule" (500 kbit/s CAN + 500000 bauds série)
  applyBusProfile(g_selectedBus);  // relais + CAN + Serial + labels
 
  vehFrameCount        = 0; // Reset des stats HUD
  vehLastPGN           = 0xFFFFFFFF;
  vehFramesSinceTick   = 0;
  vehLastTickMs        = millis();
  vehBlinkFlip         = false;
  
  if (vehHUDTimer) { // (Re)création du timer HUD si besoin
    lv_timer_del(vehHUDTimer);
    vehHUDTimer = nullptr;
  }
  vehHUDTimer = lv_timer_create(vehHUDTimer_cb, 250, NULL); // 4 Hz
 
  capVehiculeTotaleActive = true;  // Démarre la capture
  Serial.println(F("Lecture totale Vehicule: DEMARRAGE"));
  
  showScreen(SCR_VEHICULE_LECT_TOTALE); // Affiche l'écran de lecture

 }

 void btnRetourVehiculeTotale_cb(lv_event_t * e) { // Callback bouton retour depuis Véhicule Totale
  closePopupIfOpen();
  Serial.println("Retour à l'écran Véhicule depuis Véhicule Totale");
  showScreen(SCR_VEHICULE);  // revient à l'écran Véhicule
 }

 void btnRetourVehiculeAdresseSource_cb(lv_event_t * e) { // Callback du bouton retour de la page Véhicule Adresse Source
  closePopupIfOpen();
  Serial.println("Retour vers screenVehicule depuis Adresse Source");
  showScreen(SCR_VEHICULE);  
 }

 void btnVehiculeAdresseSource_cb(lv_event_t * e) { // Callback de lancement de la page de sélection d'adresse source Véhicule
  closePopupIfOpen();
  Serial.println("Bouton Adresse Source cliqué");
  showScreen(SCR_VEHICULE_SEL_AS);  
 }

 void btnImgLancementLectureSource_cb(lv_event_t * e) {

  closePopupIfOpen();
 
  if (selectedCalculateur == nullptr) { // Vérif qu'un calculateur est bien choisi
    Serial.println(F("Aucun calculateur sélectionné (SA inconnu)"));
    return;
  }
 
  g_selectedBus = BusProfile::Vehicule; // Force profil Véhicule (500k CAN / 500000 série)
  applyBusProfile(g_selectedBus);
 
  capVehiculeTotaleActive = false; // Exclusivité : coupe les autres captures/timers
  if (vehHUDTimer) { lv_timer_del(vehHUDTimer); vehHUDTimer = nullptr; }
  capEquipementTotaleActive = false;
  if (eqHUDTimer)  { lv_timer_del(eqHUDTimer);  eqHUDTimer  = nullptr; }
 
  currentSAFilter = selectedCalculateur->sa; // Paramètre de filtre SA
 
  asFrameCount = 0;  // Reset stats HUD
  asLastPGN = 0xFFFFFFFF;
  asFramesSinceTick = 0;
  asLastTickMs = millis();
  asBlinkFlip = false;
 
  if (asHUDTimer) { lv_timer_del(asHUDTimer); asHUDTimer = nullptr; } // (Re)création du timer HUD
  asHUDTimer = lv_timer_create(asHUDTimer_cb, 250, NULL);
  
  capVehiculeASActive = true; // Démarre la capture filtrée SA

  Serial.print(F("Lecture A.S. Vehicule: DEMARRAGE SA=0x"));
  Serial.println(currentSAFilter, HEX);
 
  showScreen(SCR_VEHICULE_LECT_AS); // Affiche l’écran de lecture (et mets à jour l’info calculateur comme tu faisais)
 
  if (selectedCalculateur != nullptr && labelCalculateurInfo != nullptr) {  // MAJ texte bloc d'info si label présent (tu l'avais déjà)
    static char buffer[512];
    snprintf(buffer, sizeof(buffer),
             "Adresse Source : 0x%02X\nNom : %s\nBoitier : %s\nDenomination : %s",
             selectedCalculateur->sa,
             selectedCalculateur->nom,
             selectedCalculateur->boitier,
             selectedCalculateur->denomination);
    lv_label_set_text(labelCalculateurInfo, buffer);
  }
 }

 void btnRetourVehiculeLectureAdresseSource_cb(lv_event_t * e) {
  closePopupIfOpen();

  capVehiculeASActive = false;
  if (asHUDTimer) { lv_timer_del(asHUDTimer); asHUDTimer = nullptr; }

  Serial.print(F("Lecture A.S. Vehicule: ARRET, total="));
  Serial.println(asFrameCount);

  showScreen(SCR_VEHICULE_SEL_AS);

 }


 void btnVehiculePGN_cb(lv_event_t * e) { // Bouton de lancement de la page PGN Véhicule
  closePopupIfOpen();
  Serial.println("Page PGN ouverte");
  showScreen(SCR_VEHICULE_PGN);
 }

 void btnRetourVehiculePGN_cb(lv_event_t * e) { // Callback du bouton retour pour la page PGN sélection
  closePopupIfOpen();
  Serial.println("Retour vers screenVehicule depuis PGN");
  showScreen(SCR_VEHICULE);  // Retour à l'écran Véhicule
 }

 static void kb_event_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* kb = (lv_obj_t*)lv_event_get_target(e);

  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
    if (taPGN) {
      const char* txt = lv_textarea_get_text(taPGN);
      currentPGNVehicule = atoi(txt);
      if (labelPGNTitle) {
        static char buffer[64];
        snprintf(buffer, sizeof(buffer), "Etude du PGN : %d", currentPGNVehicule);
        lv_label_set_text(labelPGNTitle, buffer);
      }
    }
    if (kb) lv_obj_del(kb);
    kbPGN = nullptr;
  }
 }

 static void kb_event_equipement_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* kb = (lv_obj_t*)lv_event_get_target(e);

  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
    if (taEquipementPGN) {
      const char* txt = lv_textarea_get_text(taEquipementPGN);
      currentPGNEquipement = atoi(txt);
      if (labelEquipementPGNTitle) {
        static char buffer[64];
        snprintf(buffer, sizeof(buffer), "Etude du PGN : %d", currentPGNEquipement);
        lv_label_set_text(labelEquipementPGNTitle, buffer);
      }
    }
    if (kb) lv_obj_del(kb);
    kbEquipementPGN = nullptr;
  }
 }

 void btnOpenKeyboardPGN_cb(lv_event_t * e) { // Bouton d'ouverture du clavier pour PGN Véicule   
  if (kbPGN != nullptr) { // Supprimer ancien clavier s'il existe
     lv_obj_del(kbPGN);
     kbPGN = nullptr;
  }
 
  kbPGN = lv_keyboard_create(lv_scr_act()); // Créer le clavier
  lv_obj_set_size(kbPGN, 400, 300);                     // taille du clavier
  lv_keyboard_set_mode(kbPGN, LV_KEYBOARD_MODE_NUMBER); // mode numérique
  lv_obj_align(kbPGN, LV_ALIGN_LEFT_MID, 20, 0); 
  lv_obj_set_style_border_width(kbPGN, 2, 0);                  // épaisseur 4 px
  lv_obj_set_style_border_color(kbPGN, lv_color_hex(0x000000), 0); // couleur noire
  lv_obj_set_style_border_opa(kbPGN, LV_OPA_COVER, 0);         // opaque       // position
  lv_keyboard_set_textarea(kbPGN, taPGN);
  lv_obj_set_style_text_font(kbPGN, &lv_font_montserrat_42, LV_PART_ITEMS);
  lv_obj_set_style_bg_color(kbPGN, lv_color_hex(0xECF255), 0);
  lv_obj_add_event_cb(kbPGN, kb_event_cb, LV_EVENT_ALL, NULL);

 }

 void btnOpenKeyboardEquipementPGN_cb(lv_event_t * e) { //  Bouton ouverture clavier Equipement PGN 
  if (kbEquipementPGN != nullptr) {
    lv_obj_del(kbEquipementPGN);
    kbEquipementPGN = nullptr;
  }

   kbEquipementPGN = lv_keyboard_create(lv_scr_act());
   lv_obj_set_size(kbEquipementPGN, 400, 300);
   lv_keyboard_set_mode(kbEquipementPGN, LV_KEYBOARD_MODE_NUMBER);
   lv_obj_align(kbEquipementPGN, LV_ALIGN_LEFT_MID, 20, 0);
   lv_keyboard_set_textarea(kbEquipementPGN, taEquipementPGN);
   lv_obj_set_style_text_font(kbEquipementPGN, &lv_font_montserrat_42, LV_PART_ITEMS);
   lv_obj_set_style_bg_color(kbEquipementPGN, lv_color_hex(0xECF255), 0);
   lv_obj_add_event_cb(kbEquipementPGN, kb_event_equipement_cb, LV_EVENT_ALL, NULL);

 }

 void imgLancementPGN_cb(lv_event_t * e) { 

  (void)e;
  closePopupIfOpen();
 
  if (taPGN) { // Récupère le PGN saisi si présent dans la zone (sécurité)
    const char* txt = lv_textarea_get_text(taPGN);
    if (txt && *txt) currentPGNVehicule = atoi(txt);
  }

  if (currentPGNVehicule < 0) {
    Serial.println(F("Aucun PGN valide saisi"));
    return;
  }
 
  g_selectedBus = BusProfile::Vehicule; // Force le profil "Véhicule"
  applyBusProfile(g_selectedBus);

  stopAllCapturesAndTimers(); // Exclusivité : coupe les autres captures/timers
  
  vehPGNFrameCount = 0; // Reset stats

  showScreen(SCR_VEHICULE_LECT_PGN);  // Affiche l'écran de lecture
  
  if (labelPGNTitle) { // MAJ du titre “Étude du PGN”
    static char buffer[64];
    snprintf(buffer, sizeof(buffer), "Etude du PGN : %d", currentPGNVehicule);
    lv_label_set_text(labelPGNTitle, buffer);
  }
  
  vehPGNFrameCount      = 0;  // Reset stats HUD
  vehPGNFramesSinceTick = 0;
  vehPGNLastTickMs      = millis();
  vehPGNBlinkFlip       = false;
 
  if (pgnHUDTimer) { lv_timer_del(pgnHUDTimer); pgnHUDTimer = nullptr; } // (Re)création du timer HUD
  pgnHUDTimer = lv_timer_create(pgnHUDTimer_cb, 250, NULL); 
  capVehiculePGNActive = true; // Démarre la capture
  Serial.println(F("Lecture PGN Vehicule: DEMARRAGE"));
   
 }

 void btnRetourLecturePGN_cb(lv_event_t * e) {
   (void)e;
   closePopupIfOpen();

   capVehiculePGNActive = false;

   pgnCntVal  = nullptr;
   pgnRateVal = nullptr;
   dotPGN     = nullptr;

  Serial.print(F("Lecture PGN Vehicule: ARRET, total="));
  Serial.println(vehPGNFrameCount);

  showScreen(SCR_VEHICULE_PGN);

 }

 void btnEquipement_cb(lv_event_t * e) { // Callback pour le bouton Equipement de la page sélection réseau CAN
  closePopupIfOpen();
  Serial.println("Bouton reseau equipement cliqué -> Page de selection du mode de lecture du reseau CAN Equipement");
  showScreen(SCR_EQUIPEMENT); 
 }

 void btnRetourEquipement_cb(lv_event_t * e) { // Callback pour le bouton Retour de la page Equipement vers la page Selection du réseau CAN
  closePopupIfOpen();
  showScreen(SCR_SEL_CAN);
 }

 void btnEquipementTotale_cb(lv_event_t * e) {  // Callback pour le bouton de lancement de la page
  closePopupIfOpen();
  showScreen(SCR_EQUIPEMENT_SEL_TOTALE);   
 }

 void btnRetourEquipementTotale_cb(lv_event_t * e) { // Callback pour le bouton retour de la page Equipement Totale
  closePopupIfOpen();
  showScreen(SCR_EQUIPEMENT);    
 }

 void imgLancementEquipementLectureTotale_cb(lv_event_t * e) {
  (void)e;
  closePopupIfOpen();

  g_selectedBus = BusProfile::Equipement;   // Profil Équipement (250 kbit/s CAN + 250000 bauds série)
  applyBusProfile(g_selectedBus);

  capVehiculeTotaleActive = false;  // Assure l'exclusivité (on coupe la capture Véhicule si elle tournait)
  if (vehHUDTimer) { lv_timer_del(vehHUDTimer); vehHUDTimer = nullptr; }

  eqFrameCount = 0; // Reset stats
  eqLastPGN = 0xFFFFFFFF;
  eqFramesSinceTick = 0;
  eqLastTickMs = millis();
  eqBlinkFlip = false;
  
  if (eqHUDTimer) { lv_timer_del(eqHUDTimer); eqHUDTimer = nullptr; }  // (Re)création du timer HUD
  eqHUDTimer = lv_timer_create(eqHUDTimer_cb, 250, NULL);
  
  capEquipementTotaleActive = true; // Démarre la capture
  Serial.println(F("Lecture totale Equipement: DEMARRAGE"));

  showScreen(SCR_EQUIPEMENT_LECT_TOTALE);  // Affiche l'écran de lecture Equipement

 }

 void btnRetourEquipementLectureTotale_cb(lv_event_t * e) {
  (void)e;
  closePopupIfOpen();
  
  capEquipementTotaleActive = false; // Stop capture + HUD
  if (eqHUDTimer) { lv_timer_del(eqHUDTimer); eqHUDTimer = nullptr; }

  Serial.print(F("Lecture totale Equipement: ARRET, total="));
  Serial.println(eqFrameCount);
  
  showScreen(SCR_EQUIPEMENT); // Retour vers le menu Équipement

 }

 void btnEquipementPGN_cb(lv_event_t * e) { // Callback de lancement du bouton vers la page de sélection PGN Equipement
  closePopupIfOpen();
  showScreen(SCR_EQUIPEMENT_SEL_PGN);
 }

 void btnRetourEquipementPGN_cb(lv_event_t * e) { // Callback de retour de la page de sélection PGN Equipement
  closePopupIfOpen();
  showScreen(SCR_EQUIPEMENT);
 }

 void imgLancementEquipementLecturePGN_cb(lv_event_t * e) {
  (void)e;
  closePopupIfOpen();
 
  if (taEquipementPGN) { // Récupère le PGN saisi si présent
    const char* txt = lv_textarea_get_text(taEquipementPGN);
    if (txt && *txt) currentPGNEquipement = atoi(txt);
  }
  if (currentPGNEquipement < 0) {
    Serial.println(F("[EQ] Aucun PGN valide saisi"));
    return;
  }
  
  g_selectedBus = BusProfile::Equipement; // Force le profil "Équipement" (250 k)
  applyBusProfile(g_selectedBus);
 
  stopAllCapturesAndTimers();  // Exclusivité
  
  eqPGNFrameCount      = 0; // Reset stats HUD
  eqPGNFramesSinceTick = 0;
  eqPGNLastTickMs      = millis();
  eqPGNBlinkFlip       = false;
 
  if (eqPgnHUDTimer) { lv_timer_del(eqPgnHUDTimer); eqPgnHUDTimer = nullptr; } // (Re)création du timer HUD
  eqPgnHUDTimer = lv_timer_create(eqPgnHUDTimer_cb, 250, NULL);
  
  showScreen(SCR_EQUIPEMENT_LECT_PGN); // Affiche l'écran de lecture

  if (labelEquipementPGNTitle) {  // MAJ du titre bas de page (déjà présent chez toi)
    static char buffer[64];
    snprintf(buffer, sizeof(buffer), "Etude du PGN : %d", currentPGNEquipement);
    lv_label_set_text(labelEquipementPGNTitle, buffer);
  }
 
  capEquipementPGNActive = true; // Démarre la capture
  Serial.println(F("[EQ] Lecture PGN: DEMARRAGE"));

 }

 void btnRetourEquipementLecturePGN_cb(lv_event_t * e) {
  (void)e;
  closePopupIfOpen();

  capEquipementPGNActive = false;
  if (eqPgnHUDTimer) { lv_timer_del(eqPgnHUDTimer); eqPgnHUDTimer = nullptr; }
  
  eqPgnCntVal = nullptr; // nettoie les pointeurs HUD
  eqPgnRateVal = nullptr;
  dotEqPGN = nullptr;

  Serial.print(F("[EQ] Lecture PGN: ARRET, total="));
  Serial.println(eqPGNFrameCount);

  showScreen(SCR_EQUIPEMENT_SEL_PGN);

 }

 static void buildWifiStatusPanel(lv_obj_t* parent); // --- Panneau d'état Wi-Fi 
 static void popup_bg_click_cb(lv_event_t * e);
 static void ta_focus_cb(lv_event_t * e);
 static void kb_wifi_event_cb(lv_event_t * e);
 
 static void purgeTopLayerTransient() {
  lv_obj_t* top = lv_layer_top();
  if (!top) return;

  uint32_t cnt = lv_obj_get_child_cnt(top);
  for (int i = (int)cnt - 1; i >= 0; --i) {
    lv_obj_t* child = lv_obj_get_child(top, i);

    // Conserver les icônes persistantes (Wi-Fi + MQTT)
    if ((imgSignalWifi && child == imgSignalWifi) ||
    (imgMqttTop   && child == imgMqttTop)     ||
    (g_versionBadge && child == g_versionBadge)) {
  continue; // on conserve le badge version
    }
    lv_obj_del(child);
  }

  kbWifi = nullptr;
  kbPGN = nullptr;
  kbEquipementPGN = nullptr;
  popupSource = nullptr;
}

 static void ensureMqttIconOnTopLayer() {
  if (!imgMqttTop) {
    imgMqttTop = lv_img_create(lv_layer_top());
    lv_img_set_src(imgMqttTop, &MosquittoLogo);
    lv_obj_align(imgMqttTop, LV_ALIGN_TOP_LEFT, 10, 10); // coin gauche
    lv_obj_add_flag(imgMqttTop, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(imgMqttTop, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_add_flag(imgMqttTop, LV_OBJ_FLAG_ADV_HITTEST); // pas cliquable
  } else {
    if (lv_obj_get_parent(imgMqttTop) != lv_layer_top()) {
      lv_obj_set_parent(imgMqttTop, lv_layer_top());
    }
    lv_obj_align(imgMqttTop, LV_ALIGN_TOP_LEFT, 10, 10);
  }
 }

 static void setMqttTopIconVisible(bool visible) {
  ensureMqttIconOnTopLayer();
  if (visible) lv_obj_clear_flag(imgMqttTop, LV_OBJ_FLAG_HIDDEN);
  else         lv_obj_add_flag(imgMqttTop,   LV_OBJ_FLAG_HIDDEN);
 }

 static void updateWifiStatusPanel() {
  if (!wifiPanel) return;

  static char bufIP[64], bufEtat[64];
 
  snprintf(bufIP, sizeof(bufIP), "IP : %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]); //  NE PAS APPELER WiFi.localIP() ICI (timer LVGL) — on lit le cache "ip"
 
  int s = wifi_status_cached; // idem : on ne lit pas WiFi.status() ici, on lit le cache
  const char* stateTxt = "Wi-Fi inactif";
  switch (s) {
    case WL_CONNECTED:       stateTxt = "Connecte au reseau Wifi"; break;
    case WL_NO_SHIELD:       stateTxt = "Module Wi-Fi absent"; break;
    case WL_IDLE_STATUS:     stateTxt = "Inactif"; break;
    case WL_NO_SSID_AVAIL:   stateTxt = "SSID introuvable"; break;
    case WL_SCAN_COMPLETED:  stateTxt = "Scan termine"; break;
    case WL_CONNECT_FAILED:  stateTxt = "Echec de connexion"; break;
    case WL_CONNECTION_LOST: stateTxt = "Connexion perdue"; break;
    case WL_DISCONNECTED:    stateTxt = "Deconnecte"; break;
    default:                 stateTxt = "Etat inconnu"; break;
  }

  snprintf(bufEtat, sizeof(bufEtat), "Etat : %s", stateTxt);

  if (lblWifiIP)   lv_label_set_text(lblWifiIP,   bufIP);
  if (lblWifiEtat) lv_label_set_text(lblWifiEtat, bufEtat);
 }
 
 static void btnWifiDisconnect_cb(lv_event_t * e) {
 
  stopAllCapturesAndTimers();    // pas obligatoire ici, mais safe si tu lisais via MQTT  / Coupe proprement l’activité réseau
 
  if (client.connected()) { // MQTT off (si connecté)
    client.disconnect();
  }

  WiFi.disconnect();   // libère DHCP/ARP et ferme la liaison  Wi-Fi off
  
  wifi_status_cached = WL_DISCONNECTED; // Mets à jour le cache immédiatement
  wifi_rssi_cached   = -127;
  ip = IPAddress(0,0,0,0);
 
  if (imgSignalWifi) { lv_obj_add_flag(imgSignalWifi, LV_OBJ_FLAG_HIDDEN); } // UI
  updateWifiStatusPanel();
 }

 static void updateWifiConnectButtonUI() {
  if (!btnWifiToggle) return;
  initClickStyles();
  
  lv_obj_remove_style_all(btnWifiToggle); // Reset visuel propre
  lv_obj_set_size(btnWifiToggle, 240, 60);
  lv_obj_align(btnWifiToggle, LV_ALIGN_BOTTOM_MID, 0, -10);
 
  lv_obj_remove_event_cb(btnWifiToggle, btnWifiConnect_cb); // Retirer d'abord tout callback existant
  lv_obj_remove_event_cb(btnWifiToggle, btnWifiDisconnect_cb);
 
  lv_obj_t *lbl = lv_obj_get_child(btnWifiToggle, 0); // Label interne (créé par creationBouton)

  applyStyleBtnRetour(btnWifiToggle); // Style structurel vert (coins, bordure)

  lv_obj_add_style(btnWifiToggle, &styleBtnBaseClick, 0); // Effet base commun
  lv_obj_add_flag(btnWifiToggle, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_text_color(btnWifiToggle, lv_color_white(), 0);

  if (wifi_status_cached == WL_CONNECTED) {    
    applyStyleBtnDanger(btnWifiToggle);  // ROUGE: "Deconnecter"                
    if (lbl) lv_label_set_text(lbl, "Deconnecter");
    lv_obj_add_event_cb(btnWifiToggle, btnWifiDisconnect_cb, LV_EVENT_CLICKED, NULL);   
    lv_obj_add_style(btnWifiToggle, &styleBtnPressedRed, LV_STATE_PRESSED); // Effet pressé spécifique rouge
  } else {    
    if (lbl) lv_label_set_text(lbl, "Se connecter"); // VERT: "Se connecter"
    lv_obj_add_event_cb(btnWifiToggle, btnWifiConnect_cb, LV_EVENT_CLICKED, NULL);    
    lv_obj_add_style(btnWifiToggle, &styleBtnPressedGreen, LV_STATE_PRESSED); // Effet pressé spécifique vert
  }
 }

 static void buildWifiStatusPanel(lv_obj_t* parent) {
  if (wifiPanel) return; // déjà créé

  // Panneau
  wifiPanel = lv_obj_create(parent);
  lv_obj_set_size(wifiPanel, 740, 320);
  lv_obj_align(wifiPanel, LV_ALIGN_TOP_MID, 0, 70);
  lv_obj_set_style_bg_color(wifiPanel, lv_color_hex(0x335E91), 0);
  lv_obj_set_style_bg_opa(wifiPanel, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(wifiPanel, 1, 0);
  lv_obj_set_style_border_color(wifiPanel, lv_color_hex(0xA89E9E), 0);//C7C7C7
  lv_obj_set_style_radius(wifiPanel, 8, 0);
  lv_obj_set_style_pad_all(wifiPanel, 12, 0);
  lv_obj_set_style_shadow_width(wifiPanel, 10, 0);
  lv_obj_set_style_shadow_color(wifiPanel, lv_color_hex(0x000000), 0);
  lv_obj_set_style_shadow_opa(wifiPanel, LV_OPA_20, 0);

  // ---------- Ligne SSID ----------
  lv_obj_t *lblSsid = lv_label_create(wifiPanel);
  lv_label_set_text(lblSsid, "Nom du reseau");
  lv_obj_set_style_text_font(lblSsid, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblSsid, lv_color_white(), 0);   // ➜ blanc
  lv_obj_align(lblSsid, LV_ALIGN_TOP_LEFT, 10, 0);

  taWifiSSID = lv_textarea_create(wifiPanel);
  lv_obj_set_size(taWifiSSID, 700, 48);
  lv_obj_align(taWifiSSID, LV_ALIGN_TOP_LEFT, 4, 40);
  lv_textarea_set_max_length(taWifiSSID, 32);
  lv_textarea_set_one_line(taWifiSSID, true);
  lv_textarea_set_placeholder_text(taWifiSSID, "SSID du reseau Wifi");
  lv_textarea_set_text(taWifiSSID, ssid); // prérempli
  lv_obj_set_style_text_font(taWifiSSID, &lv_font_montserrat_32, 0);
  lv_obj_add_event_cb(taWifiSSID, ta_focus_cb, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(taWifiSSID, ta_focus_cb, LV_EVENT_CLICKED, NULL);

  // ---------- Ligne Mot de passe ----------
  lv_obj_t *lblPass = lv_label_create(wifiPanel);
  lv_label_set_text(lblPass, "Mot de passe");
  lv_obj_set_style_text_font(lblPass, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblPass, lv_color_white(), 0);   // ➜ blanc
  lv_obj_align(lblPass, LV_ALIGN_TOP_LEFT, 10, 105);

  taWifiPass = lv_textarea_create(wifiPanel);
  lv_obj_set_size(taWifiPass, 700, 48);
  lv_obj_align(taWifiPass, LV_ALIGN_TOP_LEFT, 4, 145);
  lv_textarea_set_max_length(taWifiPass, 64);
  lv_textarea_set_one_line(taWifiPass, true);
  lv_textarea_set_password_mode(taWifiPass, true);
  lv_textarea_set_placeholder_text(taWifiPass, "Mot de passe");
  lv_textarea_set_text(taWifiPass, password); // prérempli
  lv_obj_set_style_text_font(taWifiPass, &lv_font_montserrat_32, 0);
  lv_obj_add_event_cb(taWifiPass, ta_focus_cb, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(taWifiPass, ta_focus_cb, LV_EVENT_CLICKED, NULL);

  // ---------- IP & État (affichage) ----------
  lblWifiIP = lv_label_create(wifiPanel);
  lv_obj_set_style_text_font(lblWifiIP, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblWifiIP, lv_color_white(), 0); // ➜ blanc
  lv_obj_align(lblWifiIP, LV_ALIGN_TOP_LEFT, 10, 220);

  lblWifiEtat = lv_label_create(wifiPanel);
  lv_obj_set_style_text_font(lblWifiEtat, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblWifiEtat, lv_color_white(), 0); // ➜ blanc
  lv_obj_align(lblWifiEtat, LV_ALIGN_TOP_LEFT, 10, 265);

  updateWifiStatusPanel();
 }

 void imgWifiPageConfig_cb(lv_event_t * e) {
  
  if (kbWifi) { lv_obj_del(kbWifi); kbWifi = nullptr; } // au cas où
  closePopupIfOpen();
  Serial.println("Appui bouton wifi -> page d'information Wi-Fi");
  showScreen(SCR_WIFI);
  wifiPageActive = true;
 }

 void btnRetourInfoWifi_cb(lv_event_t * e) {
  if (kbWifi) { lv_obj_del(kbWifi); kbWifi = nullptr; }
  closePopupIfOpen(); 
  showScreen(SCR_ACCUEIL);
  accueilActive = true;
  wifiPageActive = false;
  start = millis();
 }
 
 static void ensureWifiIconOnTopLayer() {
  if (!imgSignalWifi) {
    imgSignalWifi = lv_img_create(lv_layer_top());
    lv_img_set_src(imgSignalWifi, &wifiFort);   
    lv_obj_align(imgSignalWifi, LV_ALIGN_TOP_RIGHT, -10, 10); // 👇 bien dans le coin en haut à droite, sans décalage
    lv_obj_add_flag(imgSignalWifi, LV_OBJ_FLAG_HIDDEN);   
    lv_obj_add_flag(imgSignalWifi, LV_OBJ_FLAG_IGNORE_LAYOUT); // optionnel : s’assurer que le layout ne la bouge pas
    lv_obj_clear_flag(imgSignalWifi, LV_OBJ_FLAG_CLICKABLE);   
    lv_obj_add_flag(imgSignalWifi, LV_OBJ_FLAG_ADV_HITTEST);
  } else {
    if (lv_obj_get_parent(imgSignalWifi) != lv_layer_top()) {
      lv_obj_set_parent(imgSignalWifi, lv_layer_top());
    }    
    lv_obj_align(imgSignalWifi, LV_ALIGN_TOP_RIGHT, -10, 10); // 👇 réaffirmer l’alignement au cas où
  }
 }

 static void wifi_poll_cb(lv_timer_t * t) {
  static int last = -999;
  int s = wifi_status_cached; // lecture du cache

  if (s != last) {
    last = s;
    if (s == WL_CONNECTED) {
      ip = WiFi.localIP();
      lv_obj_clear_flag(imgSignalWifi, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(imgSignalWifi, LV_OBJ_FLAG_HIDDEN);
    }
  }
   
  if (imgMQTTPageConfig) { // Affichage conditionnel de l’icône MQTT sur l’accueil
    if (s == WL_CONNECTED) lv_obj_clear_flag(imgMQTTPageConfig, LV_OBJ_FLAG_HIDDEN);
  else                    lv_obj_add_flag(imgMQTTPageConfig,   LV_OBJ_FLAG_HIDDEN);
  }

  if (s == WL_CONNECTED) { // MàJ icône & panneau d'état à partir du cache (aucun appel WiFi ici)
    // Injecte le RSSI "comme si"
    // (on réutilise ta fonction, mais sans appel WiFi)
    ensureWifiIconOnTopLayer();

    static int lastBars = -2;
    static int rssiEma  = -1000;

    int rssi = wifi_rssi_cached;
    if (rssiEma < -200) rssiEma = rssi;
    else                rssiEma = (rssiEma * 3 + rssi) / 4;

    auto rssiToBars = [](int r)->int {
      if (r >= -55) return 3;
      if (r >= -65) return 2;
      if (r >= -75) return 1;
      return 0;
    };

    int bars = rssiToBars(rssiEma);
    if (bars != lastBars) {
      switch (bars) {
        case 3: lv_img_set_src(imgSignalWifi, &wifiFort);   break;
        case 2: lv_img_set_src(imgSignalWifi, &wifiBon);    break;
        case 1: lv_img_set_src(imgSignalWifi, &wifiMoyen);  break;
        default:lv_img_set_src(imgSignalWifi, &wifiFaible); break;
      }
      lv_obj_align(imgSignalWifi, LV_ALIGN_TOP_RIGHT, -10, 10);
      lastBars = bars;
    }
    // --- Icône Mosquitto (haut-gauche) en fonction de mqtt_connected_cached ---
  static int lastMqttShown = -1; // -1 force MAJ au premier passage
  int shouldShowMqtt = (wifi_status_cached == WL_CONNECTED && mqtt_connected_cached) ? 1 : 0;

  if (shouldShowMqtt != lastMqttShown) {
    setMqttTopIconVisible(shouldShowMqtt == 1);
    lastMqttShown = shouldShowMqtt;
  }
    lv_obj_clear_flag(imgSignalWifi, LV_OBJ_FLAG_HIDDEN);
  }

  updateWifiStatusPanel();  // ne lit que des labels LVGL (ok)  
  static int lastForBtn = -999; // ⚠️ Ne pas toucher le bouton chaque seconde : seulement quand l'état change
  if (s != lastForBtn) {
    updateWifiConnectButtonUI();
    lastForBtn = s;
  }
  (void)t;
 }

 void connecterWifiSTA() { // Connexion STA (client) 
  Serial.println("Initialisation du Wi-Fi en mode STA...");
  // (Optionnel) Si dispo sur ta lib : WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int status = WiFi.begin(ssid, password); // Non bloquant
  Serial.print("WiFi.begin() -> "); Serial.println(status);

  // L’état final sera suivi par wifi_poll_cb()
  updateWifiStatusPanel();
 }

 enum WifiBars { WIFI_NONE=-1, WIFI_FAIBLE=0, WIFI_MOYEN=1, WIFI_BON=2, WIFI_FORT=3 }; // ---- Barres de signal d'après RSSI (dBm) ---- Seuils courants :  -55 excellent, -65 bon, -75 moyen, en dessous faible

 static int lastBars = -2;     // force MAJ au premier passage
 static int rssiEma  = -1000;  // EMA pour lisser le RSSI

 static int rssiToBars(int rssiDbm) {
  if (rssiDbm >= -55) return WIFI_FORT;
  if (rssiDbm >= -65) return WIFI_BON;
  if (rssiDbm >= -75) return WIFI_MOYEN;
  return WIFI_FAIBLE;
 }

 void updateWifiSignalIconByRSSI() { // Met à jour l'icône globale imgSignalWifi selon le RSSI
  ensureWifiIconOnTopLayer();  // crée/replace l'image sur le layer top si besoin

  if (WiFi.status() != WL_CONNECTED) {
    // Pas connecté : cacher l’icône et réinitialiser l’état
    lv_obj_add_flag(imgSignalWifi, LV_OBJ_FLAG_HIDDEN);
    lastBars = -2;
    rssiEma  = -1000;
    return;
  }

  // Récupère RSSI (dBm). Lissage EMA 4:1 pour éviter de changer d'icône à chaque tick
  int rssi = WiFi.RSSI();              // typiquement -30 (excellent) à -90 (très faible)
  if (rssiEma < -200) rssiEma = rssi;  // init EMA au premier coup
  else                rssiEma = (rssiEma * 3 + rssi) / 4;

  int bars = rssiToBars(rssiEma);

  if (bars != lastBars) {
  switch (bars) {
    case WIFI_FORT:   lv_img_set_src(imgSignalWifi, &wifiFort);   break;
    case WIFI_BON:    lv_img_set_src(imgSignalWifi, &wifiBon);    break;
    case WIFI_MOYEN:  lv_img_set_src(imgSignalWifi, &wifiMoyen);  break;
    default:          lv_img_set_src(imgSignalWifi, &wifiFaible); break;
  }
  // 👇 rester collé en haut à droite
  lv_obj_align(imgSignalWifi, LV_ALIGN_TOP_RIGHT, -5, 5);
  lastBars = bars;
  }

  // Assure l’affichage
  lv_obj_clear_flag(imgSignalWifi, LV_OBJ_FLAG_HIDDEN);
 }

 static void ta_focus_cb(lv_event_t * e) { // Ouvre automatiquement le clavier quand on touche SSID/Password
  lv_obj_t *ta = (lv_obj_t*)lv_event_get_target(e);

  
  if (!kbWifi) { // Crée le clavier s'il n'existe pas (sur la couche TOP pour ne pas être rogné)
    kbWifi = lv_keyboard_create(lv_layer_top());
    // ====== TAILLE + POSITION ======
    lv_obj_set_size(kbWifi, 760, 260);              // plus haut que 220
    lv_obj_align(kbWifi, LV_ALIGN_BOTTOM_MID, 0, -10);

    // ====== GROSSES TOUCHES ======
    // Police des touches
    lv_obj_set_style_text_font(kbWifi, &lv_font_montserrat_32, LV_PART_ITEMS);
    // Hauteur mini des touches
    lv_obj_set_style_min_height(kbWifi, 56, LV_PART_ITEMS);
    // Espaces / marges pour un confort tactile
    lv_obj_set_style_pad_all(kbWifi, 8, LV_PART_ITEMS);
    lv_obj_set_style_pad_row(kbWifi, 6, LV_PART_ITEMS);
    lv_obj_set_style_pad_column(kbWifi, 6, LV_PART_ITEMS);

    // Gestion "OK/Annuler" -> fermer le clavier
    lv_obj_add_event_cb(kbWifi, kb_wifi_event_cb, LV_EVENT_ALL, NULL);
  }

  // Cible la textarea qui a le focus
  lv_keyboard_set_textarea(kbWifi, ta);

  // Feedback
  lv_obj_add_state(ta, LV_STATE_FOCUSED);
  lv_obj_scroll_to_view(ta, LV_ANIM_OFF);
 }

 static void kb_wifi_event_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *kb = (lv_obj_t*)lv_event_get_target(e);
  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
    if (kb) lv_obj_del(kb);
    kbWifi = nullptr;
  }
 }

 static void btnWifiConnect_cb(lv_event_t * e) { // Bouton "Se connecter" : lit les champs et lance la STA
  const char* s = taWifiSSID ? lv_textarea_get_text(taWifiSSID) : "";
  const char* p = taWifiPass ? lv_textarea_get_text(taWifiPass) : "";
  strncpy(ssid, s, sizeof(ssid)-1);        ssid[sizeof(ssid)-1] = '\0';
  strncpy(password, p, sizeof(password)-1); password[sizeof(password)-1] = '\0';

  Serial.print("Demande de connexion STA à SSID='");
  Serial.print(ssid);
  Serial.println("' ...");

  // Pause propre des captures/Timers pendant la demande de connexion
  stopAllCapturesAndTimers();

  connecterWifiSTA();
  updateWifiStatusPanel();
  updateWifiConnectButtonUI();

  if (kbWifi) { lv_obj_del(kbWifi); kbWifi = nullptr; }
 }

 static void stopAllCapturesAndTimers() {
  // Désactive les boucles de polling
  capVehiculeTotaleActive   = false;
  capEquipementTotaleActive = false;
  capVehiculeASActive       = false;
  capVehiculePGNActive = false;
  if (pgnHUDTimer) { lv_timer_del(pgnHUDTimer); pgnHUDTimer = nullptr; }
  pgnCntVal = pgnRateVal = nullptr;
  dotPGN = nullptr;
  capEquipementPGNActive = false;
  if (eqPgnHUDTimer) { lv_timer_del(eqPgnHUDTimer); eqPgnHUDTimer = nullptr; }
  eqPgnCntVal = eqPgnRateVal = nullptr;
  dotEqPGN = nullptr;


  // Timers HUD
  if (vehHUDTimer) { lv_timer_del(vehHUDTimer); vehHUDTimer = nullptr; }
  if (eqHUDTimer)  { lv_timer_del(eqHUDTimer);  eqHUDTimer  = nullptr; }
  if (asHUDTimer)  { lv_timer_del(asHUDTimer);  asHUDTimer  = nullptr; }

  // Valeurs HUD (sécurité : les callbacks checkent déjà les pointeurs)
  vehCntVal = vehRateVal = vehLastVal = nullptr;
  eqCntVal  = eqRateVal  = eqLastVal  = nullptr;
  asCntVal  = asRateVal  = asLastVal  = nullptr;
  dotVeh = dotEq = dotAS = nullptr;
 }

 static void endCAN_if_supported() {
  #if defined(ACANFD_GIGA_R1_h)
    // Certaines versions exposent end() ou endFD()
    #if __has_include(<ACANFD_GIGA_R1.h>)
      // Essaye les deux signatures sans casser la build
      // (no-op si non disponibles)
      extern ACANFD_GIGA_R1 fdcan1;
      // Ces appels seront optimisés si absents
      // (tu peux en garder un seul si tu connais ta version)
      // fdcan1.endFD();
      // fdcan1.end();
    #endif
  #endif
 }


// Fin des pages et CallBack des pages

//###########################################################################################################################################################################
//###################################################################  Création des pages de navigation du Giga Display  ######################################################################################

void createPageAccueil() {

 screenPageAccueil = lv_obj_create(NULL);

 creationTitre(screenPageAccueil, "Analyseur BUS CAN", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40);               

 lv_obj_t * sousTitre = lv_label_create(screenPageAccueil); // Création du titre modèle tracteur
 lv_label_set_text(sousTitre, "modele 6M 95/125");
 lv_obj_set_style_text_font(sousTitre, &lv_font_montserrat_32, 0);
 lv_obj_align(sousTitre, LV_ALIGN_BOTTOM_LEFT, 10, -10);  
      
 lv_obj_t * imgLogo = lv_img_create(screenPageAccueil); // Logo principal
 lv_img_set_src(imgLogo, &MSE);
 lv_obj_align(imgLogo, LV_ALIGN_CENTER, 0, 0);

 lv_obj_t * imgWifiPageConfig = lv_img_create(screenPageAccueil); 
 lv_img_set_src(imgWifiPageConfig, &Wifi);
 lv_obj_align(imgWifiPageConfig, LV_ALIGN_CENTER, -300, -70);  
 lv_obj_add_flag(imgWifiPageConfig, LV_OBJ_FLAG_CLICKABLE); 
 lv_obj_add_event_cb(imgWifiPageConfig, imgWifiPageConfig_cb, LV_EVENT_CLICKED, NULL);

 imgMQTTPageConfig = lv_img_create(screenPageAccueil); // Icône MQTT sous l’icône Wifi (cachée tant que le Wi-Fi n’est pas connecté)
 lv_img_set_src(imgMQTTPageConfig, &MQTT);
 lv_obj_align(imgMQTTPageConfig, LV_ALIGN_CENTER, -300, +70); // sous le Wifi
 lv_obj_add_flag(imgMQTTPageConfig, LV_OBJ_FLAG_CLICKABLE);
 lv_obj_add_event_cb(imgMQTTPageConfig, imgMQTTPageConfig_cb, LV_EVENT_CLICKED, NULL);
 lv_obj_add_flag(imgMQTTPageConfig, LV_OBJ_FLAG_HIDDEN); // visible seulement si WL_CONNECTED
    
 Signature(screenPageAccueil);

}

void createPageSelectionReseauCAN() {

  screenPageSelectionReseauCAN = lv_obj_create(NULL); 

  creationTitre(screenPageSelectionReseauCAN, "Selection reseau BUS CAN", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40);

  // Boutons : on utilise des callbacks dédiés pour mettre à jour l'image AVANT de naviguer
  creationBouton(screenPageSelectionReseauCAN, "CAN Vehicule", 320, 100, LV_ALIGN_LEFT_MID, 50, -60, btnSelVehicule_cb);
  creationBouton(screenPageSelectionReseauCAN, "CAN Equipement", 320, 100, LV_ALIGN_RIGHT_MID, -50, -60, btnSelEquipement_cb);

  lv_obj_t * btnRetourAcceuil = creationBouton(
      screenPageSelectionReseauCAN, "Accueil", 140, 50,
      LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourAccueil_cb );
  applyStyleBtnRetour(btnRetourAcceuil);

  // === Indicateur relais à la place de la signature ===
  imgRelaisSelection = lv_img_create(screenPageSelectionReseauCAN);
  lv_obj_align(imgRelaisSelection, LV_ALIGN_CENTER, -60, 115 );
  lv_obj_add_flag(imgRelaisSelection, LV_OBJ_FLAG_HIDDEN); // masqué tant qu'aucun choix


 // --- Carré "E/V" sous l'image de relais
 ensureBusIndicatorCreated();
 updateBusIndicator();

  // Si tu veux afficher le dernier choix au chargement :
  updateRelaisImageOnSelectionPage();
}

void createPageVehicule() {

 screenVehicule = lv_obj_create(NULL); // Création de la page
 
 creationTitre(screenVehicule, "Reseau CAN Vehicule", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40); // Création des titres
 creationTitre(screenVehicule, "Veuillez selectionner votre mode de lecture", &lv_font_montserrat_32, LV_ALIGN_TOP_MID, 0, 110);

 creationBouton(screenVehicule, "Totale", 260, 100, LV_ALIGN_LEFT_MID, 120, -15, btnVehiculeTotale_cb); // Création du bouton de lecture totale (gauche)
 creationBouton(screenVehicule, "Adresse Source", 280, 100, LV_ALIGN_RIGHT_MID, -120, -15, btnVehiculeAdresseSource_cb); // Création du bouton Par Adresse Source (droite)
 creationBouton(screenVehicule, "PGN", 260, 100, LV_ALIGN_CENTER, 0, 100, btnVehiculePGN_cb);                            // Création du bouton par PGN (centré en dessous)
   
 lv_obj_t * btnRetourVehicule = creationBouton(  screenVehicule, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourVehicule_cb ); // Bouton retour vers page Boutons 
 applyStyleBtnRetour(btnRetourVehicule);  // appliquer ton style vert

 Signature(screenVehicule); // Signature

}

void createVehiculeTotale() {

 screenVehiculeTotale = lv_obj_create(NULL);
 
 creationTitre(screenVehiculeTotale, "Selection lecture totale", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40); // Création des titres
 lblVehiculeBaud = creationTitre(screenVehiculeTotale, "Vitesse en baud : 500000", &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT, 20, 120);
 lblVehiculeSerial = creationTitre(screenVehiculeTotale, "Vitesse port serie : 500000", &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT, 20,160);
 
 
 lv_obj_t * imgBUSCANPageBouton = lv_img_create(screenVehiculeTotale); // Logo BUSCAN
 lv_img_set_src(imgBUSCANPageBouton, &BUSCAN);
 lv_obj_align(imgBUSCANPageBouton, LV_ALIGN_BOTTOM_MID, -30, -42);

 lv_obj_t * imgLancement = lv_img_create(screenVehiculeTotale); // Ajout de ton image au centre 
 lv_img_set_src(imgLancement, &Lancement);
 lv_obj_align(imgLancement, LV_ALIGN_CENTER, 250, 10);  // centré au milieu de l'écran
 lv_obj_add_flag(imgLancement, LV_OBJ_FLAG_CLICKABLE); // Rendre l'image cliquable
 lv_obj_add_event_cb(imgLancement, imgLancement_cb, LV_EVENT_CLICKED, NULL);
   
 lv_obj_t * btnRetourVehicule = creationBouton( screenVehiculeTotale, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourVehiculeTotale_cb );  
 applyStyleBtnRetour(btnRetourVehicule);  // appliquer le style vert

// Signature(screenVehiculeTotale);  // Signature

}

void createVehiculeLectureTotale() {

 screenVehiculeLectureTotale = lv_obj_create(NULL);

 static lv_style_t styleBgDark; // Style arrière-plan gris foncé 
 lv_style_init(&styleBgDark);
 lv_style_set_bg_color(&styleBgDark, lv_color_hex(0x594B47));  // gris foncé
 lv_style_set_border_width(&styleBgDark, 0);  // pas de bordure
 lv_obj_add_style(screenVehiculeLectureTotale, &styleBgDark, 0);

 static lv_style_t styleTextWhite; // Titre avec texte blanc 
 lv_style_init(&styleTextWhite);
 lv_style_set_text_color(&styleTextWhite, lv_color_white());
 lv_obj_t * titre = creationTitre(screenVehiculeLectureTotale, "Lecture totale en cours", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40);
 lv_obj_add_style(titre, &styleTextWhite, 0);
 
 lv_obj_t * hud = lv_obj_create(screenVehiculeLectureTotale); // -------- HUD compact (compteur / débit / dernier PGN + témoin) --------
 lv_obj_set_size(hud, 760, 140);
 lv_obj_align(hud, LV_ALIGN_TOP_MID, 0, 120);
 lv_obj_set_style_bg_color(hud, lv_color_hex(0x2C2C2C), 0);
 lv_obj_set_style_bg_opa(hud, LV_OPA_COVER, 0);
 lv_obj_set_style_radius(hud, 8, 0);
 lv_obj_set_style_pad_all(hud, 12, 0);
 lv_obj_set_style_border_width(hud, 1, 0);
 lv_obj_set_style_border_color(hud, lv_color_hex(0x444444), 0);

 dotVeh = lv_obj_create(hud); // Pastille
 lv_obj_set_size(dotVeh, 36, 36);
 lv_obj_set_style_radius(dotVeh, LV_RADIUS_CIRCLE, 0);
 lv_obj_set_style_bg_color(dotVeh, lv_color_hex(0x00FF66), 0);
 lv_obj_set_style_bg_opa(dotVeh, LV_OPA_20, 0);
 lv_obj_set_style_border_width(dotVeh, 2, 0);
 lv_obj_set_style_border_color(dotVeh, lv_color_hex(0x006633), 0);
 lv_obj_align(dotVeh, LV_ALIGN_LEFT_MID, 8, 0);

 lv_obj_t* col = lv_obj_create(hud); // Colonne de lignes "clé: valeur (bleu)"
 lv_obj_set_size(col, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
 lv_obj_set_style_bg_opa(col, LV_OPA_TRANSP, 0);
 lv_obj_set_style_border_width(col, 0, 0);
 lv_obj_set_style_pad_all(col, 0, 0);
 lv_obj_set_layout(col, LV_LAYOUT_FLEX);
 lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
 lv_obj_set_flex_align(col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
 lv_obj_set_style_pad_row(col, 8, 0);
 lv_obj_align(col, LV_ALIGN_LEFT_MID, 56, 0);

 makeKVRow(col, "Trames :", &vehCntVal); // 3 lignes
 makeKVRow(col, "Debit :",  &vehRateVal);
 makeKVRow(col, "Dernier PGN :", &vehLastVal);

 lv_label_set_text(vehCntVal,  "0"); // Valeurs initiales
 lv_label_set_text(vehRateVal, "0/s");
 lv_label_set_text(vehLastVal, "N/A");

 lv_obj_t * btnRetourVehiculeLecture = creationBouton( screenVehiculeLectureTotale, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourVehiculeLecture_cb ); // callback direct vers screenVehicule 
 applyStyleBtnRetour(btnRetourVehiculeLecture);

}

void createVehiculeAdresseSource() {

 screenVehiculeSelectionAdresseSource = lv_obj_create(NULL);

 creationTitre(screenVehiculeSelectionAdresseSource, "Selection d'adresse source", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40);
 lblVehiculeBaud  = creationTitre(screenVehiculeSelectionAdresseSource, "Vitesse en baud : 500000", &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT, 10, 120);
 lblVehiculeSerial = creationTitre(screenVehiculeSelectionAdresseSource, "Vitesse port serie : 500000", &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT, 10,160);
   
 btnChoisir = creationBouton( screenVehiculeSelectionAdresseSource,  "Source", 300, 100,  LV_ALIGN_CENTER, -70, 60,  openSourcePopup );  // callback pour ouvrir le popup de choix de source

 lv_obj_t * label = lv_obj_get_child(btnChoisir, 0);
 lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);  // augmente taille (32)
 lv_obj_center(label); // recentre le texte
       
 lv_obj_t * imgLancementLectureSource = lv_img_create(screenVehiculeSelectionAdresseSource); // Ajout de ton image au centre 
 lv_img_set_src(imgLancementLectureSource, &Lancement);
 lv_obj_align(imgLancementLectureSource, LV_ALIGN_CENTER, 250, 60);  // centré au milieu de l'écran
 lv_obj_add_flag(imgLancementLectureSource, LV_OBJ_FLAG_CLICKABLE); // Rendre l'image cliquable
 lv_obj_add_event_cb(imgLancementLectureSource, btnImgLancementLectureSource_cb, LV_EVENT_CLICKED, NULL);
    
 lv_obj_t * btnRetourVehiculeAdresseSource = creationBouton( screenVehiculeSelectionAdresseSource,  "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourVehiculeAdresseSource_cb ); // Bouton retour vers page Véhicule
 applyStyleBtnRetour(btnRetourVehiculeAdresseSource);

 Signature(screenVehiculeSelectionAdresseSource);  // Signature

}

void createVehiculeLectureAdresseSource() {
  screenCreateVehiculeLectureAdresseSource = lv_obj_create(NULL);

  // Fond sombre
  static lv_style_t styleBgDark;
  lv_style_init(&styleBgDark);
  lv_style_set_bg_color(&styleBgDark, lv_color_hex(0x594B47));
  lv_style_set_border_width(&styleBgDark, 0);
  lv_obj_add_style(screenCreateVehiculeLectureAdresseSource, &styleBgDark, 0);

  // Titre
  lv_obj_t * titre = creationTitre(
    screenCreateVehiculeLectureAdresseSource,
    "Lecture A.S. en cours",
    &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40
  );
  lv_obj_set_style_text_color(titre, lv_color_white(), 0);

  // Sous-titre
  lv_obj_t * descTitle = lv_label_create(screenCreateVehiculeLectureAdresseSource);
  lv_label_set_text(descTitle, "Description du logiciel :");
  lv_obj_set_style_text_font(descTitle, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(descTitle, lv_color_white(), 0);
  lv_obj_align(descTitle, LV_ALIGN_TOP_MID, 0, 100);

  // Panneau infos 2x2
  lv_obj_t * infoPanel = lv_obj_create(screenCreateVehiculeLectureAdresseSource);
  lv_obj_set_size(infoPanel, 760, 110);
  lv_obj_align(infoPanel, LV_ALIGN_TOP_MID, 0, 145);
  lv_obj_set_style_bg_color(infoPanel, lv_color_hex(0x2C2C2C), 0);
  lv_obj_set_style_bg_opa(infoPanel, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(infoPanel, 8, 0);
  lv_obj_set_style_pad_all(infoPanel, 12, 0);
  lv_obj_set_style_border_width(infoPanel, 1, 0);
  lv_obj_set_style_border_color(infoPanel, lv_color_hex(0x444444), 0);

  static lv_coord_t cols[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
  static lv_coord_t rows[] = { LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST };
  lv_obj_set_grid_dsc_array(infoPanel, cols, rows);

  // Couleurs & police
  const lv_color_t colLabel = lv_color_white();
  const lv_color_t colValue = lv_color_hex(0x00BFFF); // bleu ciel

  auto makeKVCell = [&](const char* k, const char* v, int row, int col) {
    // conteneur horizontal pour "clé : valeur"
    lv_obj_t* cell = lv_obj_create(infoPanel);
    lv_obj_set_size(cell, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(cell, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cell, 0, 0);
    lv_obj_set_style_pad_all(cell, 0, 0);
    lv_obj_set_layout(cell, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(cell, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cell, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(cell, 8, 0); // petit espace entre clé et valeur

    lv_obj_set_grid_cell(cell, LV_GRID_ALIGN_START, col, 1, LV_GRID_ALIGN_START, row, 1);

    // clé
    lv_obj_t* kl = lv_label_create(cell);
    lv_label_set_text(kl, k);
    lv_obj_set_style_text_font(kl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(kl, colLabel, 0);

    // valeur (bleu ciel)
    lv_obj_t* vl = lv_label_create(cell);
    lv_label_set_text(vl, v);
    lv_obj_set_style_text_font(vl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(vl, colValue, 0);
  };

  // Valeurs (ou placeholders) — valeurs seules (sans le libellé) pour les mettre en bleu
  char vAS[16]   = "--";
  char vNom[48]  = "--";
  char vBoi[64]  = "--";

  if (selectedCalculateur != nullptr) {
    snprintf(vAS,  sizeof(vAS),  "0x%02X", selectedCalculateur->sa);
    snprintf(vNom, sizeof(vNom), "%s",     selectedCalculateur->nom);
    snprintf(vBoi, sizeof(vBoi), "%s",     selectedCalculateur->boitier);
  }

  // Ligne 1 : (AS / Nom)
  makeKVCell("Adresse Source :", vAS,  0, 0);
  makeKVCell("Nom :",            vNom, 0, 1);

  // Ligne 2 : (Boîtier / —)
  makeKVCell("Boitier :", vBoi, 1, 0);
  makeKVCell("",         "",   1, 1);  // cellule droite vide

  // -------- HUD "Adresse Source" --------
 lv_obj_t * hudAS = lv_obj_create(screenCreateVehiculeLectureAdresseSource);
 lv_obj_set_size(hudAS, 760, 140);
 lv_obj_align(hudAS, LV_ALIGN_TOP_MID, 0, 270);
 lv_obj_set_style_bg_color(hudAS, lv_color_hex(0x2C2C2C), 0);
 lv_obj_set_style_bg_opa(hudAS, LV_OPA_COVER, 0);
 lv_obj_set_style_radius(hudAS, 8, 0);
 lv_obj_set_style_pad_all(hudAS, 12, 0);
 lv_obj_set_style_border_width(hudAS, 1, 0);
 lv_obj_set_style_border_color(hudAS, lv_color_hex(0x444444), 0);

 // Pastille témoin
 dotAS = lv_obj_create(hudAS);
 lv_obj_set_size(dotAS, 36, 36);
 lv_obj_set_style_radius(dotAS, LV_RADIUS_CIRCLE, 0);
 lv_obj_set_style_bg_color(dotAS, lv_color_hex(0x00FF66), 0);
 lv_obj_set_style_bg_opa(dotAS, LV_OPA_20, 0);
 lv_obj_set_style_border_width(dotAS, 2, 0);
 lv_obj_set_style_border_color(dotAS, lv_color_hex(0x006633), 0);
 lv_obj_align(dotAS, LV_ALIGN_LEFT_MID, 8, 0);

 // Colonne "clé : valeur (valeur en BLEU CIEL)"
 lv_obj_t* colAS = lv_obj_create(hudAS);
 lv_obj_set_size(colAS, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
 lv_obj_set_style_bg_opa(colAS, LV_OPA_TRANSP, 0);
 lv_obj_set_style_border_width(colAS, 0, 0);
 lv_obj_set_style_pad_all(colAS, 0, 0);
 lv_obj_set_layout(colAS, LV_LAYOUT_FLEX);
 lv_obj_set_flex_flow(colAS, LV_FLEX_FLOW_COLUMN);
 lv_obj_set_flex_align(colAS, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
 lv_obj_set_style_pad_row(colAS, 8, 0);
 lv_obj_align(colAS, LV_ALIGN_LEFT_MID, 56, 0);

 // 3 lignes (les *valeurs* récupérées dans asCntVal/asRateVal/asLastVal sont bleues via makeKVRow)
 makeKVRow(colAS, "Trames :",      &asCntVal);
 makeKVRow(colAS, "Debit :",       &asRateVal);
 makeKVRow(colAS, "Dernier PGN :", &asLastVal);

 // Valeurs initiales
 lv_label_set_text(asCntVal,  "0");
 lv_label_set_text(asRateVal, "0/s");
 lv_label_set_text(asLastVal, "N/A");

  // Bouton Retour
  lv_obj_t * btnRetour = creationBouton(
    screenCreateVehiculeLectureAdresseSource,
    "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10,
    btnRetourVehiculeLectureAdresseSource_cb
  );
  applyStyleBtnRetour(btnRetour);
}

void createVehiculePGN() {

 screenVehiculePGN = lv_obj_create(NULL);

 creationTitre(screenVehiculePGN, "Selection par ciblage", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40);
 lv_obj_t * titre = creationTitre(screenVehiculePGN, "Veuillez selectionner votre PGN", &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT, 100, 100);

 taPGN = lv_textarea_create(screenVehiculePGN); // Zone de texte PGN
 lv_obj_set_size(taPGN, 240, 100);
 lv_obj_set_style_text_align(taPGN, LV_TEXT_ALIGN_CENTER, 0);  
 lv_obj_set_style_pad_top(taPGN, 25, 0);   // ajuste la valeur pour centrer verticalement
 lv_obj_set_style_pad_bottom(taPGN, 25, 0);

 lv_obj_align_to(taPGN, titre, LV_ALIGN_OUT_RIGHT_MID, -140, 90);
 lv_textarea_set_max_length(taPGN, 5);
 lv_textarea_set_one_line(taPGN, true);
 lv_textarea_set_placeholder_text(taPGN, "PGN");
 lv_obj_set_style_text_font(taPGN, &lv_font_montserrat_42, 0);
   
 creationBouton( screenVehiculePGN, "Clavier", 250, 100, LV_ALIGN_CENTER, -160, 40, btnOpenKeyboardPGN_cb ); // Bouton clavier
  
 lv_obj_t * imgLancementLecturePGN = lv_img_create(screenVehiculePGN); //  Ajout de ton image au centre
 lv_img_set_src(imgLancementLecturePGN, &Lancement);
 lv_obj_align(imgLancementLecturePGN, LV_ALIGN_CENTER, 180, 115);
 lv_obj_add_flag(imgLancementLecturePGN, LV_OBJ_FLAG_CLICKABLE);
 lv_obj_add_event_cb(imgLancementLecturePGN, imgLancementPGN_cb, LV_EVENT_CLICKED, NULL);
   
 lv_obj_t * btnRetourVehiculePGN = creationBouton( screenVehiculePGN, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourVehiculePGN_cb ); // Bouton retour
 applyStyleBtnRetour(btnRetourVehiculePGN);
 
 Signature(screenVehiculePGN); // Signature

}

void createVehiculeLecturePGN() {

 screenVehiculeLecturePGN = lv_obj_create(NULL);

 lv_obj_t * titre = creationTitre(screenVehiculeLecturePGN, "Lecture par PGN en cours", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40);
 lv_obj_set_style_text_color(titre, lv_color_hex(0xFFFFFF), 0);
 lv_obj_t * sousTitre = creationTitre(screenVehiculeLecturePGN, "Analyse de la data", &lv_font_montserrat_32, LV_ALIGN_TOP_MID, 0, 110);
 lv_obj_set_style_text_color(sousTitre, lv_color_hex(0xFFFFFF), 0);

 static lv_style_t styleBgDark;
 lv_style_init(&styleBgDark);
 lv_style_set_bg_color(&styleBgDark, lv_color_hex(0x594B47));
 lv_style_set_border_width(&styleBgDark, 0);
 lv_obj_add_style(screenVehiculeLecturePGN, &styleBgDark, 0);

 lv_obj_t * zonePGN = lv_obj_create(screenVehiculeLecturePGN);  // Zone d'affichage des 8 octets PGN sur fond blanc
 lv_obj_set_size(zonePGN, 770, 100);               // largeur/hauteur de la zone
 lv_obj_align(zonePGN, LV_ALIGN_CENTER, 0, -10);    // placement au centre un peu plus bas
 lv_obj_set_style_bg_color(zonePGN, lv_color_hex(0xFFFFFF), 0);
 lv_obj_set_style_border_color(zonePGN, lv_color_hex(0x000000), 0);
 lv_obj_set_style_border_width(zonePGN, 3, 0);
 lv_obj_set_style_text_font(zonePGN, &lv_font_montserrat_32, 0);
 
 lblVehPGNBytes = lv_label_create(zonePGN); // Label global pour MAJ temps réel
 lv_label_set_text(lblVehPGNBytes, "000.000.000.000.000.000.000.000");
 lv_obj_set_style_text_color(lblVehPGNBytes, lv_color_hex(0x000000), 0);
 lv_obj_set_style_text_font(lblVehPGNBytes, &lv_font_montserrat_42, 0);
 lv_obj_center(lblVehPGNBytes);

 lv_obj_t * hudPGN = lv_obj_create(screenVehiculeLecturePGN); // -------- HUD PGN (pastille + Trames + Débit sur le même axe) --------
 lv_obj_set_size(hudPGN, 760, 140);

 lv_obj_align(hudPGN, LV_ALIGN_TOP_MID, 0, 280);  // ↓ Descend un peu plus le HUD (augmenter si tu veux encore plus bas)
 lv_obj_set_style_bg_color(hudPGN, lv_color_hex(0x2C2C2C), 0);
 lv_obj_set_style_bg_opa(hudPGN, LV_OPA_COVER, 0);
 lv_obj_set_style_radius(hudPGN, 8, 0);
 lv_obj_set_style_pad_all(hudPGN, 12, 0);
 lv_obj_set_style_border_width(hudPGN, 1, 0);
 lv_obj_set_style_border_color(hudPGN, lv_color_hex(0x444444), 0);

 lv_obj_t* row = lv_obj_create(hudPGN); // Conteneur principal en ligne pour aligner tout "sur le même axe"
 lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
 lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
 lv_obj_set_style_border_width(row, 0, 0);
 lv_obj_set_style_pad_all(row, 0, 0);
 lv_obj_set_layout(row, LV_LAYOUT_FLEX);
 lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
 lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
 lv_obj_set_style_pad_column(row, 24, 0); // espace entre éléments
 lv_obj_center(row);

 dotPGN = lv_obj_create(row); // Pastille (à gauche)
 lv_obj_set_size(dotPGN, 36, 36);
 lv_obj_set_style_radius(dotPGN, LV_RADIUS_CIRCLE, 0);
 lv_obj_set_style_bg_color(dotPGN, lv_color_hex(0x00FF66), 0);
 lv_obj_set_style_bg_opa(dotPGN, LV_OPA_20, 0);
 lv_obj_set_style_border_width(dotPGN, 2, 0);
 lv_obj_set_style_border_color(dotPGN, lv_color_hex(0x006633), 0);

 makeKVRow(row, "Trames :", &pgnCntVal); // "Trames :" (au centre, dans l'axe de la pastille)

 makeKVRow(row, "Debit :", &pgnRateVal); // "Debit :" (à droite, même axe)

 
 lv_label_set_text(pgnCntVal,  "0"); // Valeurs initiales
 lv_label_set_text(pgnRateVal, "0/s");
    
 labelPGNTitle = lv_label_create(screenVehiculeLecturePGN); // Nouveau titre bas droite (vide au départ)
 lv_label_set_text(labelPGNTitle, "Etude du PGN :"); // vide au départ
 lv_obj_set_style_text_color(labelPGNTitle, lv_color_hex(0xFFFFFF), 0);
 lv_obj_set_style_text_font(labelPGNTitle, &lv_font_montserrat_32, 0);
 lv_obj_align(labelPGNTitle, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
 
 lv_obj_t * btnRetourVehiculeLecturePGN = creationBouton(screenVehiculeLecturePGN, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourLecturePGN_cb); // Bouton retour en bas à gauche
 applyStyleBtnRetour(btnRetourVehiculeLecturePGN);

}

void createPageEquipement() {
   
 screenEquipement = lv_obj_create(NULL);  // Création de la page

 creationTitre(screenEquipement, "Reseau CAN Equipement", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40); // Création des titres
 creationTitre(screenEquipement, "Veuillez selectionner votre mode de lecture", &lv_font_montserrat_32, LV_ALIGN_TOP_MID, 0, 110);
  
 creationBouton(screenEquipement, "Totale", 260, 100, LV_ALIGN_LEFT_MID, 120, 35, btnEquipementTotale_cb);  // Création du bouton de lecture totale (gauche)
 creationBouton(screenEquipement, "PGN", 260, 100, LV_ALIGN_RIGHT_MID, -120, 35, btnEquipementPGN_cb);  // Création du bouton par PGN (centré en dessous)
 
 lv_obj_t * btnRetourEquipement = creationBouton( screenEquipement, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourEquipement_cb ); // Bouton retour
 applyStyleBtnRetour(btnRetourEquipement);
  
 Signature(screenEquipement); // Signature

}

void createEquipementSelectionTotale() {

 screenEquipementSelectionTotale = lv_obj_create(NULL);

 creationTitre(screenEquipementSelectionTotale, "Selection lecture totale", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40); // Création des titres
 lblEquipBaud  = creationTitre(screenEquipementSelectionTotale, "Vitesse en baud : 250000", &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT, 20, 120);
 lblEquipSerial = creationTitre(screenEquipementSelectionTotale, "Vitesse port serie : 250000", &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT, 20,160);
  
 lv_obj_t * imgBUSCANPageBouton = lv_img_create(screenEquipementSelectionTotale); // Logo BUSCAN
 lv_img_set_src(imgBUSCANPageBouton, &BUSCAN);
 lv_obj_align(imgBUSCANPageBouton, LV_ALIGN_BOTTOM_MID, -30, -42);

 lv_obj_t * imgLancementEquipementLectureTotale = lv_img_create(screenEquipementSelectionTotale);  
 lv_img_set_src(imgLancementEquipementLectureTotale, &Lancement);
 lv_obj_align(imgLancementEquipementLectureTotale, LV_ALIGN_CENTER, 250, 10);  
 lv_obj_add_flag(imgLancementEquipementLectureTotale, LV_OBJ_FLAG_CLICKABLE); 
 lv_obj_add_event_cb(imgLancementEquipementLectureTotale, imgLancementEquipementLectureTotale_cb, LV_EVENT_CLICKED, NULL);

 lv_obj_t * btnRetourEquipementSelectionTotale = creationBouton( screenEquipementSelectionTotale, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourEquipementTotale_cb ); // Bouton retour
 applyStyleBtnRetour(btnRetourEquipementSelectionTotale);
   
// Signature(screenEquipementSelectionTotale); // Signature

}

void createEquipementLectureTotale() {

 screenEquipementLectureTotale = lv_obj_create(NULL);

 lv_obj_t * titreEquipementLectureTotale = creationTitre(screenEquipementLectureTotale, "Lecture totale en cours", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40);
 lv_obj_set_style_text_color(titreEquipementLectureTotale, lv_color_hex(0xFFFFFF), 0);

 static lv_style_t styleBgDark;
 lv_style_init(&styleBgDark);
 lv_style_set_bg_color(&styleBgDark, lv_color_hex(0x594B47));
 lv_style_set_border_width(&styleBgDark, 0);
 lv_obj_add_style(screenEquipementLectureTotale, &styleBgDark, 0);
 
 lv_obj_t * hudEq = lv_obj_create(screenEquipementLectureTotale); // -------- HUD Équipement --------
 lv_obj_set_size(hudEq, 760, 140);
 lv_obj_align(hudEq, LV_ALIGN_TOP_MID, 0, 120);
 lv_obj_set_style_bg_color(hudEq, lv_color_hex(0x2C2C2C), 0);
 lv_obj_set_style_bg_opa(hudEq, LV_OPA_COVER, 0);
 lv_obj_set_style_radius(hudEq, 8, 0);
 lv_obj_set_style_pad_all(hudEq, 12, 0);
 lv_obj_set_style_border_width(hudEq, 1, 0);
 lv_obj_set_style_border_color(hudEq, lv_color_hex(0x444444), 0);

 dotEq = lv_obj_create(hudEq); // Pastille
 lv_obj_set_size(dotEq, 36, 36);
 lv_obj_set_style_radius(dotEq, LV_RADIUS_CIRCLE, 0);
 lv_obj_set_style_bg_color(dotEq, lv_color_hex(0x00FF66), 0);
 lv_obj_set_style_bg_opa(dotEq, LV_OPA_20, 0);
 lv_obj_set_style_border_width(dotEq, 2, 0);
 lv_obj_set_style_border_color(dotEq, lv_color_hex(0x006633), 0);
 lv_obj_align(dotEq, LV_ALIGN_LEFT_MID, 8, 0);
 
 lv_obj_t* colEq = lv_obj_create(hudEq); // Colonne KV
 lv_obj_set_size(colEq, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
 lv_obj_set_style_bg_opa(colEq, LV_OPA_TRANSP, 0);
 lv_obj_set_style_border_width(colEq, 0, 0);
 lv_obj_set_style_pad_all(colEq, 0, 0);
 lv_obj_set_layout(colEq, LV_LAYOUT_FLEX);
 lv_obj_set_flex_flow(colEq, LV_FLEX_FLOW_COLUMN);
 lv_obj_set_flex_align(colEq, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
 lv_obj_set_style_pad_row(colEq, 8, 0);
 lv_obj_align(colEq, LV_ALIGN_LEFT_MID, 56, 0);
 
 makeKVRow(colEq, "Trames :", &eqCntVal); // 3 lignes
 makeKVRow(colEq, "Debit :",  &eqRateVal);
 makeKVRow(colEq, "Dernier PGN :", &eqLastVal);
  
 lv_label_set_text(eqCntVal,  "0"); // Valeurs initiales
 lv_label_set_text(eqRateVal, "0/s");
 lv_label_set_text(eqLastVal, "N/A");


 lv_obj_t * btnRetourEquipementLectureTotale = creationBouton( screenEquipementLectureTotale, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourEquipementLectureTotale_cb ); // Bouton retour
 applyStyleBtnRetour(btnRetourEquipementLectureTotale);

}

void createEquipementSelectionPGN() {

 screenEquipementSelectionPGN = lv_obj_create(NULL);
       
 creationTitre(screenEquipementSelectionPGN, "Selection par ciblage", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40); // Titre
 lv_obj_t * titreEquipement = creationTitre(screenEquipementSelectionPGN, "Veuillez selectionner votre PGN", &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT, 100, 100);
   
 taEquipementPGN = lv_textarea_create(screenEquipementSelectionPGN); // Zone de texte PGN
 lv_obj_set_size(taEquipementPGN, 240, 100);
 lv_obj_set_style_text_align(taEquipementPGN, LV_TEXT_ALIGN_CENTER, 0);  
 lv_obj_set_style_pad_top(taEquipementPGN, 25, 0);   // ajuste la valeur pour centrer verticalement
 lv_obj_set_style_pad_bottom(taEquipementPGN, 25, 0);
 lv_obj_align_to(taEquipementPGN, titreEquipement, LV_ALIGN_OUT_RIGHT_MID, -140, 90);
 lv_textarea_set_max_length(taEquipementPGN, 5);
 lv_textarea_set_one_line(taEquipementPGN, true);
 lv_textarea_set_placeholder_text(taEquipementPGN, "PGN");
 lv_obj_set_style_text_font(taEquipementPGN, &lv_font_montserrat_42, 0);

 creationBouton( screenEquipementSelectionPGN, "Clavier", 250, 100, LV_ALIGN_CENTER, -160, 40, btnOpenKeyboardEquipementPGN_cb ); // Bouton clavier
  
 lv_obj_t * imgLancementEquipementLecturePGN = lv_img_create(screenEquipementSelectionPGN); // Ajout de ton image au centre
 lv_img_set_src(imgLancementEquipementLecturePGN, &Lancement);
 lv_obj_align(imgLancementEquipementLecturePGN, LV_ALIGN_CENTER, 180, 115);
 lv_obj_add_flag(imgLancementEquipementLecturePGN, LV_OBJ_FLAG_CLICKABLE);
 lv_obj_add_event_cb(imgLancementEquipementLecturePGN, imgLancementEquipementLecturePGN_cb, LV_EVENT_CLICKED, NULL);

 lv_obj_t* btnRetour = creationBouton( screenEquipementSelectionPGN, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourEquipementPGN_cb ); // Bouton retour
 applyStyleBtnRetour(btnRetour);

}

void createEquipementLecturePGN() {

 screenEquipementLecturePGN = lv_obj_create(NULL);
 
 lv_obj_t * titreEquipementLecturePGN = creationTitre(screenEquipementLecturePGN, "Lecture par PGN en cours", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 40);
 lv_obj_set_style_text_color(titreEquipementLecturePGN, lv_color_hex(0xFFFFFF), 0);
 lv_obj_t * sousTitreEquipementLecturePGN = creationTitre(screenEquipementLecturePGN, "Analyse de la data", &lv_font_montserrat_32, LV_ALIGN_TOP_MID, 0, 110);
 lv_obj_set_style_text_color(sousTitreEquipementLecturePGN, lv_color_hex(0xFFFFFF), 0);

 static lv_style_t styleBgDark;
 lv_style_init(&styleBgDark);
 lv_style_set_bg_color(&styleBgDark, lv_color_hex(0x594B47));
 lv_style_set_border_width(&styleBgDark, 0);
 lv_obj_add_style(screenEquipementLecturePGN, &styleBgDark, 0);

 lv_obj_t * zonePGNEquipementLecturePGN = lv_obj_create(screenEquipementLecturePGN); // Zone d'affichage des 8 octets PGN fond blanc
 lv_obj_set_size(zonePGNEquipementLecturePGN, 770, 100);               // largeur/hauteur de la zone
 lv_obj_align(zonePGNEquipementLecturePGN, LV_ALIGN_CENTER, 0, -10);    // placement au centre un peu plus bas
 lv_obj_set_style_bg_color(zonePGNEquipementLecturePGN, lv_color_hex(0xFFFFFF), 0);
 lv_obj_set_style_border_color(zonePGNEquipementLecturePGN, lv_color_hex(0x000000), 0);
 lv_obj_set_style_border_width(zonePGNEquipementLecturePGN, 3, 0);
 lv_obj_set_style_text_font(zonePGNEquipementLecturePGN, &lv_font_montserrat_32, 0);
 
 lblEqPGNBytes = lv_label_create(zonePGNEquipementLecturePGN); // Label GLOBAL pour MAJ live
 lv_label_set_text(lblEqPGNBytes, "000.000.000.000.000.000.000.000");
 lv_obj_set_style_text_color(lblEqPGNBytes, lv_color_hex(0x000000), 0);
 lv_obj_set_style_text_font(lblEqPGNBytes, &lv_font_montserrat_42, 0);
 lv_obj_center(lblEqPGNBytes);
 
 lv_obj_t * hudEqPGN = lv_obj_create(screenEquipementLecturePGN); // -------- HUD PGN Équipement (pastille + Trames + Débit alignés) --------
 lv_obj_set_size(hudEqPGN, 760, 140);
 lv_obj_align(hudEqPGN, LV_ALIGN_TOP_MID, 0, 280); // descend le HUD
 lv_obj_set_style_bg_color(hudEqPGN, lv_color_hex(0x2C2C2C), 0);
 lv_obj_set_style_bg_opa(hudEqPGN, LV_OPA_COVER, 0);
 lv_obj_set_style_radius(hudEqPGN, 8, 0);
 lv_obj_set_style_pad_all(hudEqPGN, 12, 0);
 lv_obj_set_style_border_width(hudEqPGN, 1, 0);
 lv_obj_set_style_border_color(hudEqPGN, lv_color_hex(0x444444), 0);

 lv_obj_t* rowEq = lv_obj_create(hudEqPGN); // Ligne flex (même axe)
 lv_obj_set_size(rowEq, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
 lv_obj_set_style_bg_opa(rowEq, LV_OPA_TRANSP, 0);
 lv_obj_set_style_border_width(rowEq, 0, 0);
 lv_obj_set_style_pad_all(rowEq, 0, 0);
 lv_obj_set_layout(rowEq, LV_LAYOUT_FLEX);
 lv_obj_set_flex_flow(rowEq, LV_FLEX_FLOW_ROW);
 lv_obj_set_flex_align(rowEq, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
 lv_obj_set_style_pad_column(rowEq, 24, 0);
 lv_obj_center(rowEq);
 
 dotEqPGN = lv_obj_create(rowEq); // Pastille
 lv_obj_set_size(dotEqPGN, 36, 36);
 lv_obj_set_style_radius(dotEqPGN, LV_RADIUS_CIRCLE, 0);
 lv_obj_set_style_bg_color(dotEqPGN, lv_color_hex(0x00FF66), 0);
 lv_obj_set_style_bg_opa(dotEqPGN, LV_OPA_20, 0);
 lv_obj_set_style_border_width(dotEqPGN, 2, 0);
 lv_obj_set_style_border_color(dotEqPGN, lv_color_hex(0x006633), 0);

 makeKVRow(rowEq, "Trames :", &eqPgnCntVal); // "Trames :" (au centre) 
 makeKVRow(rowEq, "Debit :", &eqPgnRateVal); // "Debit :" (à droite)

 lv_label_set_text(eqPgnCntVal,  "0"); // Valeurs initiales
 lv_label_set_text(eqPgnRateVal, "0/s");

 labelEquipementPGNTitle = lv_label_create(screenEquipementLecturePGN);
 lv_label_set_text(labelEquipementPGNTitle, "Etude du PGN :");
 lv_obj_set_style_text_font(labelEquipementPGNTitle, &lv_font_montserrat_32, 0);
 lv_obj_set_style_text_color(labelEquipementPGNTitle, lv_color_white(), 0);
 lv_obj_align(labelEquipementPGNTitle, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
 
 lv_obj_t * btnRetourEquipementLecturePGN = creationBouton(screenEquipementLecturePGN, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourEquipementLecturePGN_cb);
 applyStyleBtnRetour(btnRetourEquipementLecturePGN);

}

void createConfigurationWifi() {

 screenConfigurationWifi = lv_obj_create(NULL);
 lv_obj_set_style_bg_color(screenConfigurationWifi, lv_color_hex(0xE3D5D3), 0);
 lv_obj_set_style_bg_opa(screenConfigurationWifi, LV_OPA_COVER, 0);

 creationTitre(screenConfigurationWifi, "Configuration Wifi", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 15);
 
 buildWifiStatusPanel(screenConfigurationWifi); // Panneau avec champs dedans
 updateWifiStatusPanel();
 
 btnWifiToggle = creationBouton( screenConfigurationWifi, "Se connecter", 240, 60, LV_ALIGN_BOTTOM_MID, 0, -10, NULL ); // le callback est posé par updateWifiConnectButtonUI()

 applyStyleBtnRetour(btnWifiToggle);
 updateWifiConnectButtonUI();  // règle libellé/style/callback selon l'état courant

  lv_obj_t * btnRetourInfoWifi = creationBouton( screenConfigurationWifi, "Retour", 140, 50, LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourInfoWifi_cb);
  applyStyleBtnRetour(btnRetourInfoWifi);

}

void createConfigurationMqtt() {
  screenConfigurationMqtt = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(screenConfigurationMqtt, lv_color_hex(0xE3D5D3), 0);
  lv_obj_set_style_bg_opa(screenConfigurationMqtt, LV_OPA_COVER, 0);

  creationTitre(screenConfigurationMqtt, "Configuration MQTT", &lv_font_montserrat_42, LV_ALIGN_TOP_MID, 0, 15);

  buildMqttStatusPanel(screenConfigurationMqtt); // champs + labels
  updateMqttStatusPanel();

  // Bouton Connexion/Déconnexion
  btnMqttToggle = creationBouton(screenConfigurationMqtt, "Se connecter", 240, 60,
                                 LV_ALIGN_BOTTOM_MID, 0, -10, NULL);
  applyStyleBtnRetour(btnMqttToggle);  // style vert
  updateMqttConnectButtonUI();         // met le libellé & callbacks selon l’état

  // Bouton Retour (vert)
  lv_obj_t * btnRetour = creationBouton(screenConfigurationMqtt, "Retour", 140, 50,
                                        LV_ALIGN_BOTTOM_LEFT, 10, -10, btnRetourInfoMqtt_cb);
  applyStyleBtnRetour(btnRetour);

  // Petit timer pour rafraîchir l’état (connecté/déconnecté) sans bloquer
 // if (mqttTimer) { lv_timer_del(mqttTimer); mqttTimer = nullptr; }
//  mqttTimer = lv_timer_create(mqtt_poll_cb, 1000, NULL);
}
// _________________________________________  SETUP  _________________________________________

void setup() {

 pinMode(relaisSelectionBUSCAN, OUTPUT);

 Serial.begin(serialBaudFor(g_selectedBus));  // démarre directement au bon débit
 applyBusProfile(g_selectedBus);              // init relais + CAN + série
 setRelayForCurrentSelection();

 lv_init();
 Display.begin();
 TouchDetector.begin();
 ensureVersionBadgeOnTop();
 initClickStyles();

 
 ensureWifiIconOnTopLayer();  // Icône Wi-Fi top layer + timer
 wifiTimer = lv_timer_create(wifi_poll_cb, 1000, NULL);
 
client.setCallback(callback);
ensureMqttIconOnTopLayer();

  
 showScreen(SCR_ACCUEIL); // Démarre directement sur l’écran d’accueil (créé à la demande)
 start = millis();
 accueilActive = true;

}

// _________________________________________  LOOP  _________________________________________ 

void loop() {
 
 lv_timer_handler();
  
 pollVehiculeTotale();
 pollEquipementTotale();
 pollVehiculeAS();
 pollVehiculePGN();
 pollEquipementPGN();

  if (wifi_status_cached == WL_CONNECTED) {
    if (client.connected()) {
      client.loop();      // OK uniquement si déjà connecté
    } else {
      reconnect();        // Tentative périodique sinon
    }
  }
 mqtt_connected_cached = client.connected();

  if (wifi_status_cached == WL_CONNECTED && client.connected()) { // Publication périodique des vitesses toutes les 5 s
  unsigned long nowPub = millis();
    if (nowPub - lastSpeedPub >= SPEED_PUB_PERIOD) {
    lastSpeedPub = nowPub;
    mqtt_publish_speeds();
    }
  }
  
 unsigned long now = millis();
  if (now - last_wifi_poll_ms >= 1000) {   // Poll Wi-Fi hors LVGL (toutes les 1000 ms)
    last_wifi_poll_ms = now;
    wifi_status_cached = WiFi.status();
    if (wifi_status_cached == WL_CONNECTED) {
      wifi_rssi_cached = WiFi.RSSI();
      ip = WiFi.localIP();  // AJOUT ICI : rafraîchir l'IP côté "loop"
    } else {
      wifi_rssi_cached = -127;
    }
  }

  if (accueilActive && !wifiPageActive && !mqttPageActive && millis() - start > 5000) {
  showScreen(SCR_SEL_CAN);
  accueilActive = false;
}
  

 delay(3); // évite la busy-loop et améliore la réactivité

}
 
// __________________________ Fonctions _____________________________


