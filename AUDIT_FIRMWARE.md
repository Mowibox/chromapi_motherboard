# Audit firmware — Chromapi motherboard (STM32G431KBT6)

Périmètre : `firmware/chromapi_stm32_core/Core` (~3,5 kLOC applicatives). Les fichiers CubeMX et `Drivers/` sont lus mais pas audités.
Sauf mention contraire, les chemins sont relatifs à `firmware/chromapi_stm32_core/`.
Légende : **FAIT** = observé dans le code ou le build · **RISQUE** = défaut probable, non reproduit · **AVIS** = préférence.
Audit en lecture seule (date : 2026-10-24). Rien n'a été compilé ni flashé : les chiffres mémoire viennent du `Debug/` existant (build du 2026-10-22).

---

## 1. Verdict

L'architecture est saine pour sa taille : super-loop + DMA/IDLE, une machine à états servo correcte, des parseurs bornés, et l'appli tient dans les blocs `USER CODE`.
Le vrai problème, c'est **l'absence de filet de sécurité** : pas de watchdog, pas de failsafe si le Pi se tait, et des timeouts bloquants de 1 s, voire infinis, dans la boucle.
Il y a aussi **un bug numérique** : la racine carrée CORDIC sort de sa plage dès que l'accélération dépasse ~1,7 g. Or c'est exactement ce que produit un impact de patte.
Les saccades viennent probablement de la façon dont les consignes sont appliquées (en échelon, sans interpolation, avec un cycle qui saute quand un servo manque), et non d'un manque d'ordonnancement.
RTOS : **pas maintenant**. Rust : **non pour ce projet**, mais intéressant en side-project.

---

## 2. Tableau des constats

### Critique

| Sévérité | Fichier:ligne | Problème | Correctif proposé |
|---|---|---|---|
| Critique | `Core/Inc/stm32g4xx_hal_conf.h:49`, `Core/Src/main.c:862-869` | **FAIT** : aucun watchdog (`HAL_IWDG_MODULE_ENABLED` commenté). `Error_Handler` coupe les IRQ et boucle à l'infini. **RISQUE** : tout blocage (voir les deux lignes suivantes) fige le robot dans sa pose, couple actif, sans retour possible. | IWDG ~100 ms, rafraîchi en fin de boucle. Dans `Error_Handler`, tenter un SyncWrite couple OFF puis laisser l'IWDG resetter. |
| Critique | `Core/Src/bridge.c:128-133`, `Core/Src/bridge.c:144-182` | **FAIT** : aucun timeout côté Pi. La dernière consigne `SET_POSITIONS` reste appliquée indéfiniment. Si le Pi plante ou si le câble se débranche en pleine marche, rien ne se passe côté MCU. | Mémoriser `HAL_GetTick()` à chaque trame valide (`parse_rx_buffer`). Au-delà de ~200 ms, passer en couple OFF (ou maintien) et allumer une LED rouge. |
| Critique | `Core/Inc/ina226.h:58`, `Core/Src/ina226.c:205,210`, `Core/Src/main.c:383-386` | **FAIT** : les accès I2C INA226 ont un timeout de **1000 ms** et sont appelés depuis la boucle principale à 10 Hz. Chaque lecture fait Transmit + Receive, et il y en a 3 (`bridge.c:228-230`). **RISQUE** : un bus I2C bloqué (SDA tenu bas par du bruit servo) arrête la boucle jusqu'à ~6 s : plus de cycle servo, plus d'IMU, débordement du RX bridge. | Timeout de 2 ms, compteur d'erreurs, et récupération du bus (`HAL_I2C_DeInit`/`Init`) après N échecs. |
| Critique | `Core/Src/bmi088.c:143,162,180-181,193-194,225,247` | **FAIT** : SPI IMU en `HAL_MAX_DELAY`, avec en plus un `while(HAL_SPI_GetState()…)` sans borne. Les retours de `BMI088_Read*` sont ignorés (`main.c:269-270`). **RISQUE** : un blocage SPI fige le système pour toujours (pas d'IWDG). | Timeout de 1 ms, et vérifier les retours (en cas d'échec, sauter la mise à jour Mahony). |

### Majeur

| Sévérité | Fichier:ligne | Problème | Correctif proposé |
|---|---|---|---|
| Majeur | `Core/Src/cordic_sqrt.c:13,24,27` (✅ corrigé) | **FAIT, confirmé par AN5325 §2.1** : l'entrée de la racine CORDIC doit vérifier `x ≤ 2,34` (limite de convergence, quel que soit SCALE) et `2^-n·(x + 0,25) < 1` (pas de débordement). Le code écrit `x/4` en `SCALE_0` : le débordement impose `x/4 < 0,75`, donc `x = ‖a‖² < 3 g²`, soit **‖a‖ < 1,73 g**. Au-delà de 4 g², la conversion `int32` sature (`:24`). L'accéléro est en ±3 g (`bmi088.c:57`) et un impact de patte dépasse ce seuil. **Aucun réglage SCALE ne relève la limite** : avec `SCALE_2`, la convergence plafonne la valeur réelle à 2,34, donc ‖a‖ ≤ 1,53 g. Seule une réduction de plage logicielle (diviser par 4^k, multiplier le résultat par 2^k) irait plus loin. Et AN5325 p. 11 constate qu'en flottant simple précision, le coût de conversion float↔entier annule le gain CORDIC pour la racine. | **Remplacé par `sqrtf`** (`mahony.c:3,56,85`) et `cordic_sqrt.*` supprimés. `MX_CORDIC_Init` reste (code généré) : décocher CORDIC dans le `.ioc` pour l'enlever. |
| Majeur | `Core/Src/bridge.c:149-157`, `Core/Src/main.c:369-376` | **FAIT** : seul `TARGET_POS` (2 octets) est écrit. La consigne du Pi part telle quelle au prochain cycle de 200 Hz. Aucun registre vitesse/accélération n'est écrit par le firmware. **RISQUE (piste n°1 des saccades)** : une politique RL à 30-50 Hz donne des **échelons** que le STS3215 suit à accélération max. Le cycle n'est pas phase-locké sur l'arrivée des consignes, ce qui ajoute 0 à 5 ms de gigue. | Mesurer d'abord : fréquence d'envoi du Pi, `cycle_us`, `seq` (déjà dans le snapshot). Ensuite, soit interpoler linéairement sur le MCU à 200 Hz entre deux consignes, soit régler ACC/GOAL_SPEED une fois au boot. |
| Majeur | `Core/Src/main.c:203-234`, `:285`, `:101` | **FAIT** : un servo absent au boot est exclu à vie de la SyncRead (`active_servo_ids`). À l'inverse, un servo présent au boot qui décroche ensuite coûte, à chaque cycle, un timeout de 4-5 ms sur un budget de 5 ms. **RISQUE** : le cycle tombe à ~100 Hz avec de la gigue, et les écritures de position aussi, d'où des saccades. | Compter les absences par servo. Le retirer de la SyncRead après N échecs, et retenter un ping toutes les ~500 ms. |
| Majeur | `Core/Src/sts3215_hal.c:98-101` | **FAIT** : si un octet LEN est corrompu (par ex. 0xF0), `ParseReply` renvoie `FRAME_SHORT` et la boucle fait `break`, en attente d'octets qui n'arriveront pas. **RISQUE** : toutes les réponses suivantes de la SyncRead sont ignorées jusqu'au timeout, et une seule erreur de bit fait perdre le cycle entier. | Si `buf[3] > STS3215_REPLY_DATA_MAX + 2`, traiter comme `BAD_HEADER` (`offset++`) au lieu de `break`. |
| Majeur | `Core/Src/stm32g4xx_hal_msp.c:566`, `Core/Src/sts3215_hal.c:61-67,310` | **FAIT** : le DMA RX servo est en mode **NORMAL**. Après chaque IDLE (une par réponse de servo), la réception est arrêtée par la HAL puis réarmée dans l'ISR (`prv_arm_rx`), avec le `memcpy` vers `rx_accum` (`:295`). **RISQUE** : si le servo suivant commence à répondre pendant la fenêtre de réarmement (Return Delay faible), on perd ses premiers octets, donc son header, donc la réponse (voir aussi la ligne précédente). C'est cohérent avec le « bug d'état DMA/UART » évoqué. | DMA **circulaire** + `ReceiveToIdle` jamais arrêté, avec un parsing incrémental sur l'index d'écriture. Le pattern existe déjà dans le bridge (`bridge.c:432-438,447-458,512-514`). Cela supprime aussi `rx_accum` (256 B de RAM). |
| Majeur | `Core/Src/bridge.c:22,447-458,512-514`, `Core/Src/stm32g4xx_hal_msp.c:469`, `Core/Src/main.c:672` | **FAIT** : le RX bridge est un DMA circulaire de 256 B à 2 Mbaud, soit **1,28 ms** de trafic au maximum, sans détection de débordement. **FAIT** : les commandes servo de config bloquent la boucle jusqu'à ~225 ms (`handle_cmd_set_servo_id` : `wait_servo_idle(55)` + 3×txn + `HAL_Delay(20)` en `:312` + ping). **RISQUE** : pendant ces blocages (ou un blocage I2C), un trafic soutenu fait faire un tour complet au buffer. On a alors `head == tail` : les données sont perdues **sans erreur visible**. | Compter les octets reçus via les événements HT/TC/IDLE et les comparer aux octets consommés pour détecter le wrap. Plus simple encore : ne jamais bloquer dans `parse_rx_buffer` (voir RTOS §3, alternative légère). |
| Majeur | `Core/Src/bridge.c:13-20`, `Core/Src/bridge.c:58`, `Core/Src/main.c:78-83,130-136` | **FAIT** : dépendance circulaire. `bridge.c` fait des `extern` sur 8 globals de `main.c` (dont le handle servo et des tampons de réponse), et `main.c` écrit dans `g_robot_state`, défini dans `bridge.c`. Le « bridge » contient en réalité le protocole Pi, les transactions servo bloquantes, la logique LED batterie et le cache INA226. **AVIS** : c'est le principal frein à la compréhension en 15 min. | Extraire `servo_bus.c` (cycle, poll, `claim/servo_txn`, `on_reply`, `g_robot_state`). `bridge.c` ne garde que le protocole et appelle une API. Pas besoin de plus de couches. |
| Majeur | `Core/Src/main.c:824-828`, `Core/Src/stm32g4xx_it.c` (aucun handler EXTI), `Core/Src/bmi088.c:46-50,54,96-100,107` | **FAIT** : les broches INT de l'IMU sont configurées en EXTI, mais aucun IRQ n'est activé ni géré. L'IMU est lue par TIM6 à 400 Hz, sans synchro avec l'ODR (accéléro 400 Hz, gyro 1 kHz) : échantillons dupliqués ou sautés, gigue. Les chip ID sont lus puis ignorés (`return 0` commenté), et le retour de `BMI088_Init` est ignoré (`main.c:289`). Une IMU absente passe donc inaperçue. | Vérifier le chip ID et le signaler (LED + flag dans le snapshot). Option : déclencher la lecture sur l'INT data-ready de l'accéléro. |
| Majeur | `Core/Src/mahony.c:20-34` | **FAIT** : la calibration soustrait 9,81 sur z uniquement, ce qui suppose le robot à plat au boot. Elle bloque aussi 5 s (500 × `HAL_Delay(10)`). **RISQUE** : un boot sur un sol incliné inscrit l'inclinaison comme « biais accéléro », et le quaternion reste biaisé pour toute la session. | Ne calibrer que le biais gyro. Pour l'accéléro, l'erreur Mahony + `Ki` gère déjà le biais résiduel. Sinon, normaliser `asum` et ne retirer que l'écart de norme. |

### Mineur

| Sévérité | Fichier:ligne | Problème | Correctif proposé |
|---|---|---|---|
| Mineur | `Core/Src/main.c:276,293`, `:287`, `:344` | **FAIT** : `CordicSqrt_Init` est appelé deux fois. Les retours de `AutoFox_INA226_Init` et de `HAL_TIM_Base_Start_IT` sont ignorés. | Un seul appel, et vérifier les retours. |
| Mineur | `Core/Src/ws2812b.c:69-75`, `Core/Src/main.c:388-391`, `Core/Src/bridge.c:60,208,223` | **FAIT** : `led_render` *annule* le transfert en cours et *abandonne* la nouvelle demande. Une commande LED du Pi qui tombe pendant le rendu batterie (2 Hz) est donc perdue. **FAIT** : `g_led_override_until_ms` est écrit mais jamais lu, donc l'override de 5 s annoncé n'existe pas. | Un flag « dirty » re-rendu au prochain tour de boucle. Lire l'override dans `Bridge_UpdateBatteryLed`, ou supprimer la variable. |
| Mineur | `Core/Src/ws2812b.c:12`, `Core/Inc/ws2812b.h:1` | **FAIT** : `NUM_BPP 3` (WS2812B) alors que le header s'appelle `_LED_DRIVER_SK6812` et que le contexte parle de SK6812. La schématique (`hardware/stm32_mcu_core.kicad_sch`) indique un WS2812B-2020 pour la LED de statut. | Vérifier la référence de l'anneau. SK6812 RGBW ⇒ `NUM_BPP 4`, sinon couleurs décalées. |
| Mineur | `Core/Src/sts3215_hal.c:6-34` vs `:174-179` | **FAIT** : le commentaire d'état dit que la RX est armée dans `TxCplt`, alors que le code l'arme dans `SendFrame`, avant la TX. La documentation a divergé du code sur le point le plus délicat. | Mettre le diagramme à jour. |
| Mineur | `Core/Inc/sts3215_protocol.h:13`, `.cproject:37` | **FAIT** : la macro de config HAL `STS3215_HAL_NO_WEAK_OVERRIDE` est définie dans le header *protocole* (censé être pur), et le `-D` du `.cproject` ne s'applique qu'à l'assembleur. Ça marche par accident d'include. | Supprimer le bloc weak-override (voir ponytail n°1), ce qui rend la macro inutile. |
| Mineur | `Core/Src/main.c:415-419` | **FAIT** : SYSCLK = HSI16 × PLL, sans quartz sur la schématique MCU. UART à 2 Mbaud (Pi) et 1 Mbaud (servos). **RISQUE** faible : dérive du HSI en température. | Vérifier la précision du HSI16 dans DS12589. Si des erreurs de trame bridge apparaissent à chaud, c'est une piste. |
| Mineur | `hardware/peripherals.kicad_sch`, `Core/Inc/stm32g4xx_hal_conf.h` | **FAIT** : un MAX98357A (ampli I2S) et un ICS-43434 (micro I2S) sont sur la carte, mais **aucun code audio**, et le module HAL I2S n'est pas activé. Le contexte « audio I2S » ne correspond pas au firmware. | Rien à corriger. À noter pour la décision RTOS (§3). |
| Mineur | `Core/Src/bridge.c:58`, `Core/Src/main.c:130-136` | **AVIS** : `g_robot_state` est écrit en ISR (USART2, prio 0) et lu dans la boucle (`send_frame` → `memcpy`) sans `volatile` ni barrière. C'est correct aujourd'hui (séquencement par `hservo.state` volatile + appels inter-TU), mais ça casse silencieusement si on active LTO. | `__DMB()` ou `__asm volatile("" ::: "memory")` avant `Bridge_SendStateSnapshot`. |
| Mineur | `Core/Src/sts3215_hal.c:191-196` | **FAIT** : l'état `STS3215_HAL_STATE_ERROR` est positionné puis écrasé par `IDLE` trois lignes plus loin. C'est un état mort. | Supprimer l'état. |

### Points positifs vérifiés (pour calibrer la sévérité)

- **FAIT** : le bus servo n'utilise pas un transceiver RS485. Ce sont des buffers 74LVC1G125/126 pilotés par `USART2_DE` (`hardware/stm32_mcu_core.kicad_sch`). Le DE est **matériel** (`HAL_RS485Ex_Init`, assert/deassert 8/16 de bit, `main.c:730`) et le RX a un pull-up (`msp.c:535`). Le turnaround ne dépend donc pas du logiciel : c'est le bon choix.
- **FAIT** : les parseurs sont bornés. Bridge : taille max 64 et CRC16 (`bridge.c:390,402`). Servo : copie clampée (`sts3215_protocol.c:408`). Le buffer de trame est remis à zéro s'il est plein (`bridge.c:452`).
- **FAIT** : la file TX du bridge est protégée par section critique (`bridge.c:115-121`), le relancement depuis l'ISR est correct, et le débordement est compté (`g_bridge_tx_overflow_count`).
- **FAIT** : contournement correct et commenté du comportement HAL « erreur FE/NE bloquante en DMA » (`sts3215_hal.c:54-67`) : EIE est coupé, le checksum rattrape.

### Priorités NVIC (FAIT)

`USART2` = 0 (`msp.c:576`), `USART1` = 1 (`msp.c:496`), DMA1 ch1-5 = 5 (`main.c:764-776`), `TIM6` = 6 (`msp.c:343`), `TIM4` = 7 (`msp.c:329`), SysTick = 15 (`hal_conf.h:184`).
C'est cohérent : le bus servo est prioritaire, puis le Pi, puis les tâches. Seul bémol : le parsing des réponses servo tourne dans l'ISR IDLE à prio 0. Il peut retarder le remplissage des demi-buffers LED (prio 5, deadline ~30 µs par LED) et provoquer des glitchs de couleur. Ce n'est pas un problème moteur.

### Budget mémoire (FAIT, `Debug/chromapi_stm32_core.map`, build `-O0`)

| Région | Utilisé | Total |
|---|---|---|
| Flash (.text 0x116b0 + .rodata 0x4c4 + .data 0x1d4) | ≈ 72,9 KB | 128 KB (57 %) |
| RAM statique (.data 0x1d4 + .bss 0x122c) | ≈ 5,0 KB | 32 KB |
| Heap / pile min. (`STM32G431KBTX_FLASH.ld:41-42`) | 0,5 KB / 1 KB | — |

La plus grosse frame de pile (`Core/Src/*.su`) est `Bridge_DispatchPendingWrites` à 288 B. Le pire chemin (boucle → `parse_rx_buffer` → `handle_cmd_*` → `servo_txn`) reste bien sous 1 KB, plus le stacking des ISR. **La marge est très large.**
`_printf_float`, `_scanf_float` et `_strtod_l` sont liés (`.cproject:28`, option nano printf float) alors qu'aucun format flottant n'est utilisé : ~10 KB de flash pour rien.

### Outillage (FAIT)

- Build : projet CubeIDE managé uniquement (`.cproject`). Pas de CMake ni de Makefile versionné (`Debug/` n'est pas suivi). Un build CI headless est possible via `stm32cubeide --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild`, mais lourd.
- Warnings : `-Wall` seulement, pas de `-Wextra` (`Debug/Core/Src/subdir.mk`). Debug en `-O0`, Release en `-Os` : les timings mesurés en Debug ne représentent pas la Release.
- Tests : aucun, alors que `sts3215_protocol.h:6` annonce « Fully unit-testable on host ». La couche protocole et le parseur bridge sont du C pur et testables avec `gcc` sur PC en ~50 lignes.
- CI : aucune (`.github/` absent).

### Audit sur-ingénierie (`/ponytail:ponytail-audit`)

1. delete : bloc weak-override + registre `s_instance` / `STS3215_HAL_RegisterInstance` (`Core/Src/sts3215_hal.c:43-47,140-143,337-368`), + la macro (`Core/Inc/sts3215_protocol.h:13`). `main.c:173-201` route déjà les callbacks.
2. delete : `cordic_sqrt.c/.h` + `MX_CORDIC_Init` + module HAL CORDIC, remplacés par `sqrtf` (natif FPU, voir le constat Majeur).
3. delete : builders STS3215 jamais appelés (`BuildMotionCmd`, `BuildRegWrite*`, `BuildAction`, `BuildWriteRaw`, `BuildEnable/Disable/CenterTorque`, `BuildReset*`, conversions `Steps/Deg/Raw*`) dans `Core/Src/sts3215_protocol.c`. À garder si l'outil Pi en a besoin un jour : YAGNI.
4. delete : API INA226 inutilisée (`Hibernate`, `Wakeup`, `SetOperatingMode`, `ConfigureAlertPinTrigger`, `ResetAlertPin`, `ConfigureVoltageConversionTime`, `ConfigureNumSampleAveraging`, `Debug_GetConfigRegister`, `GetShuntVoltage_uV`) dans `Core/Src/ina226.c`.
5. delete : chemins DMA BMI088 (`Core/Src/bmi088.c:262-329`) + `readingAcc/Gyr`, `accTxBuf/gyrTxBuf` (`bmi088.c:21-22,79,124`). Le SPI n'a aucun DMA configuré.
6. delete : `led_set_all_RGB`, `led_set_RGBW`, `led_set_all_RGBW` et les branches `#if NUM_BPP == 4` (`Core/Src/ws2812b.c`), une fois la variante LED tranchée.
7. delete : `STS3215_HAL_GetState`, `rx_received_len`, `STS3215_HAL_STATE_ERROR`, `g_led_override_until_ms` (ou l'implémenter).
8. native : décocher « Use float with printf » (`.cproject:28`) : −~10 KB de flash.

net : ~−450 lignes possibles, −1 périphérique (CORDIC), 0 dépendance externe à retirer.

### Infos manquantes (non inventées)

- Le *Return Delay* réellement programmé dans les servos. `STS3215_DEFAULT_RETURN_DELAY` vaut 0 dans `sts3215_regs.h:200`, mais rien n'indique que les servos ont été configurés ainsi. Le budget « ~3 ms pour 12 servos » (`main.c:283-284`) en dépend.
- La fréquence d'envoi des consignes par le Pi.
- La fréquence I2C effective (`Timing = 0x10F02A89` à PCLK1 = 160 MHz, non recalculée ici).
- La précision du HSI16 sur la plage de température réelle.

---

## 3. RTOS : exploratoire, avis **PAS MAINTENANT**

**Activités concurrentes réelles (FAIT)** :

| Activité | Rythme | Échéance | Mécanisme actuel |
|---|---|---|---|
| Cycle servo (SyncWrite + SyncRead) | 200 Hz | 5 ms, molle | TIM6 → flag → boucle, transferts DMA/ISR |
| IMU + Mahony | 400 Hz | 2,5 ms, molle | TIM6 → flag → SPI polling |
| Bridge Pi RX/TX | événementiel | ~ms | DMA circulaire + file TX en ISR |
| INA226 | 10 Hz | aucune | I2C polling |
| LEDs | 2 Hz + à la demande | aucune | TIM4 PWM + DMA |
| Audio | — | — | **inexistant** |

Il n'y a qu'**une seule ressource critique**, le bus servo, déjà sérialisée par une machine à états. Le travail CPU est faible : IMU + Mahony ≈ quelques dizaines de µs, le reste est du DMA. Le super-loop tient les échéances **tant que rien ne bloque**.

**Les bugs actuels viennent-ils d'un manque d'ordonnancement ? Non.**
- Réarmement du DMA RX servo, état HAL, parsing qui s'arrête sur un LEN corrompu : ce sont des bugs de **synchronisation ISR/DMA**. Un RTOS ne les corrige pas, et les ISR resteraient identiques.
- Timeouts de 1 s ou infinis : une tâche bloquée 6 s reste bloquée 6 s. Un RTOS sauve les autres tâches, mais le watchdog reste nécessaire.
- Consignes en échelon : c'est une question de **politique de commande**, sans rapport avec l'ordonnancement.
- **Risque d'aggravation** : aujourd'hui, `handle_cmd_ping_servo`, `scan_active_servos` et le cycle accèdent au bus servo *séquentiellement* par construction. Dans des tâches séparées, il faudrait un mutex sur le bus. Un oubli donnerait des courses bien plus dures à déboguer que le code actuel.

**Ce qu'un RTOS apporterait de plus** :
- Les commandes de config bloquantes (`SET_SERVO_ID`, `READ_SERVO_REGISTER`…) dans une tâche basse priorité, sans geler l'IMU ni le RX bridge.
- Des timeouts naturels (`xQueueReceive(q, …, pdMS_TO_TICKS(5))`) et une période IMU propre (`vTaskDelayUntil`).
- Un cadre pour l'audio I2S le jour où il arrive (DMA double buffer + traitement de 1-5 ms qui ne doit pas retarder le servo).
- La mesure de charge par tâche (`vTaskGetRunTimeStats`), et la détection de débordement de pile.

**Ce qu'il coûterait de plus** :
- RAM : ~1 KB de pile par tâche × 4-5 + TCB + tas ≈ 6 KB. C'est faisable : 5 KB utilisés sur 32 KB.
- Flash : environ 6-10 KB pour le noyau FreeRTOS.
- La base de temps HAL doit quitter SysTick (TIM dédié), comme le recommande ST quand FreeRTOS utilise SysTick (UM1722 « Developing applications on STM32Cube with RTOS »).
- Les ISR qui appellent l'API (`…FromISR`) doivent avoir une priorité numériquement ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY` ([FreeRTOS — RTOS for ARM Cortex-M](https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)). Or USART2 est à 0 aujourd'hui : il faudrait soit la descendre, soit ne rien appeler de l'OS depuis elle.
- Le debug devient plus dur : débordements de pile, inversion de priorité, tâches affamées. Sur un projet où le temps doit aller à la locomotion, c'est un vrai coût.

**Alternative légère (recommandée)** :
1. Transformer les commandes de config en machines à états non bloquantes, pilotées par la boucle (le pattern existe déjà avec `g_writes_due` et `SERVO_TXN_*`).
2. Une petite file d'événements (`ring buffer` d'enums) remplie par les ISR et vidée par la boucle, à la place des flags `volatile bool` dispersés.
3. Watchdog + timeouts courts.
On obtient 90 % du bénéfice pour ~100 lignes.

**Ce qui ferait changer l'avis** : l'arrivée d'un traitement audio non trivial (micro ICS-43434 avec détection de mot-clé, par exemple), ou de plus de 2 activités longues et indépendantes qui doivent coexister.

---

## 4. Rust : exploratoire, avis **NON pour ce projet aujourd'hui** (oui pour un side-project d'apprentissage)

**Support réel du G431** ([docs.embassy.dev — embassy-stm32, flavor stm32g431kb](https://docs.embassy.dev/embassy-stm32/git/stm32g431kb/index.html)) :

| Besoin | embassy-stm32 | Remarque |
|---|---|---|
| UART DMA + IDLE | oui (`usart`, `RingBufferedUartRx`) | La RX circulaire + IDLE est le modèle *par défaut*, exactement le correctif proposé ci-dessus. |
| DE matériel (half-duplex servo) | à vérifier | Configuration du DE USART à valider dans la source `usart`, sinon accès PAC direct. |
| SPI (BMI088), I2C (INA226) | oui | Drivers async, avec timeouts via `embassy_time::with_timeout`. |
| PWM + DMA (LEDs) | oui (`timer`) | Il existe des exemples WS2812 avec PWM DMA pour STM32 sous embassy. |
| CORDIC | oui (module `cordic`) | Sans objet : `sqrtf` suffit. |
| I2S | module `i2s` présent | Pour l'audio futur. Le brochage I2S en LQFP32 est à vérifier dans DS12589 (le G431 a 2 SPI avec I2S multiplexé, [fiche produit ST](https://www.st.com/en/microcontrollers-microprocessors/stm32g431kb.html)). |
| Watchdog | oui (`wdg`) | — |

Le support matériel n'est donc **pas** le facteur bloquant.

**Quels bugs observés le typage ou l'ownership aurait-il empêchés ? Précisément :**
- **Handle UART servo partagé** : `hservo` est global et `extern` dans `bridge.c:13`, utilisé à la fois par la boucle, `scan_active_servos` et les `handle_cmd_*`. En Rust, il n'y aurait qu'un seul propriétaire, et les commandes de config passeraient par lui. L'interférence actuelle (`claim_servo_bus` qui écrase `g_servo_txn = POLL`, `bridge.c:284`, et fait perdre le snapshot du cycle) serait impossible à écrire.
- **Données ISR↔boucle sans `volatile` ni barrière** (`g_robot_state`) : en Rust, cela ne compile pas sans `critical_section::Mutex`, un `Signal` ou un atomique.
- **Buffers DMA** : un buffer prêté au DMA ne peut pas être lu ni copié tant que le transfert n'est pas rendu, ce qui interdit par construction le motif « réarmer puis copier ».
- **Valeurs de retour ignorées** (`BMI088_Init`, `INA226_Init`, `HAL_CORDIC_Calculate`, `BMI088_Read*`) : `Result` est `#[must_use]`, ce qui donne au minimum un warning et force un traitement explicite.
- **Conversion float→int hors plage** : en Rust, `as` sature de façon définie, et `cordic` exposerait des types à plage contrainte. On aurait un comportement défini, mais toujours faux.

**Ce que Rust n'aurait PAS empêché**, c'est-à-dire la majorité des constats : plage CORDIC, timeouts de 1 s, absence de watchdog et de failsafe, consignes en échelon, calibration à plat, servo exclu à vie, buffer bridge trop petit. Ce sont des **erreurs de conception**, pas de mémoire ni de concurrence.

**Coût** :
- Réécriture complète d'environ 3,5 kLOC, plus la reconstruction de l'init d'horloge et du pinout à la main (perte de CubeMX et de son `.ioc`).
- Courbe d'apprentissage async/embassy (executors, `'static`, `StaticCell`, traits HAL) : plusieurs semaines pour être productif.
- Ce temps est pris sur la locomotion, le RL et le sim2real, qui sont le vrai sujet du robot.

**Incrémental ?** Techniquement oui : une `staticlib` Rust `no_std` peut être liée dans le projet CubeIDE via FFI, par exemple pour la couche pure (protocole STS3215, parseur bridge). Mais c'est justement **la partie déjà saine et bornée**, et le gain serait proche de zéro. Ce que Rust améliore vraiment (ownership des périphériques, DMA, partage ISR) demande que le Rust *possède* les périphériques. En pratique, c'est donc presque **tout ou rien**.

**Bilan exploratoire** :

| | Ce que ça apporterait | Ce que ça coûterait |
|---|---|---|
| RTOS | Isolation des commandes longues, timeouts natifs, cadre pour l'audio | ~6 KB de RAM, mutex sur le bus servo, NVIC à revoir, debug plus dur |
| Rust/embassy | Ownership des périphériques et des buffers DMA, `Result` obligatoire, RX ring-buffered par défaut | Réécriture totale, perte de CubeMX, semaines d'apprentissage, et ~70 % des constats non couverts |

Pour apprendre Rust embarqué : réécrire **ce firmware** en side-project sur une deuxième carte, une fois le C stabilisé. C'est un excellent terrain d'apprentissage, parce que la spec est connue.

---

## 5. Top 5 des actions (ratio impact/effort)

| # | Action | Effort | Impact |
|---|---|---|---|
| 1 | Remplacer `CordicSqrt` par `sqrtf` (et supprimer CORDIC) | 5 min | Orientation correcte sous impact (bug réel à chaque pas) |
| 2 | IWDG ~100 ms + failsafe perte du lien Pi (couple OFF ou maintien après ~200 ms sans trame) | ~1 h | Sécurité : plus de robot figé couple actif, ni de consigne fantôme |
| 3 | Timeouts I2C/SPI à 1-2 ms + vérifier chip ID et retours d'init | ~30 min | Supprime les blocages de 1 s à ∞ de la boucle |
| 4 | Instrumenter (fréquence du Pi, `cycle_us`, `seq`, `servo_fresh_mask` déjà dans le snapshot), puis interpoler les consignes à 200 Hz ou régler ACC/GOAL_SPEED, et gérer les servos absents | ~½ j | Piste principale des **saccades** |
| 5 | Parser servo : resync sur LEN invalide + DMA RX circulaire sans réarmement, avec un test hôte `gcc` de `STS3215_ParseReply` | ~½ j | Fiabilité de la SyncRead, et premier test automatisé du repo |

---

### Sources externes

- ST, AN5325 Rev 8 *How to use the CORDIC to perform mathematical functions on STM32 MCUs* : §2.1 p. 5-6 (domaine de la racine, `x ≤ 2,34`, contrainte `2^-n·(x+0,25) < 1`) et p. 11 (aucun gain pour la racine en flottant). Voir aussi [AN5325 — Getting started with the CORDIC accelerator](https://www.st.com/resource/en/application_note/dm00614795-getting-started-with-the-cordic-accelerator-using-stm32cubeg4-mcu-package-stmicroelectronics.pdf) et [la formation ST CORDIC](https://www.st.com/resource/en/product_training/STM32G4-Peripheral-Cordic_Coprocessor_CORDIC.pdf).
- ST, [fiche produit STM32G431KB](https://www.st.com/en/microcontrollers-microprocessors/stm32g431kb.html) (3 SPI dont 2 avec I2S multiplexé) et datasheet DS12589.
- ST, UM1722 *Developing applications on STM32Cube with RTOS* (base de temps HAL hors SysTick avec FreeRTOS).
- [FreeRTOS — Running the RTOS on an ARM Cortex-M core](https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4) (`configMAX_SYSCALL_INTERRUPT_PRIORITY`).
- [docs.embassy.dev — embassy-stm32, flavor stm32g431kb](https://docs.embassy.dev/embassy-stm32/git/stm32g431kb/index.html) : modules `usart`, `dma`, `spi`, `i2c`, `timer`, `cordic`, `i2s`, `wdg` présents.
- ARM, *Cortex-M4 Technical Reference Manual*, FPU instruction timings (`VSQRT.F32` ≈ 14 cycles).

---

## Suivi des corrections (2026-10-24)

| # Top 5 | État | Changement |
|---|---|---|
| 1 | ✅ | `sqrtf` remplace `CordicSqrt` (`Core/Src/mahony.c`). `cordic_sqrt.c/.h` supprimés. |
| 2 | ✅ | Les trois niveaux du cahier de concepts (§20.5, « Chien de garde »), côté STM32 : **IWDG** ~500 ms, démarré après l'init, gelé sous debugger (`main.c`, `iwdg_start`) ; **chien de garde de liaison** de 100 ms, soit 10 trames à 100 Hz : purge des consignes et couple OFF sur les 12 servos au cycle suivant (`bridge.c`, `Bridge_CheckLink`, compteur `g_bridge_link_lost_count`) ; **relâche explicite au boot** par diffusion de couple OFF (`main.c`, `release_all_servos`), et cause du reset IWDG signalée sur ITM. Le niveau « intention » relève du Pi. |
| 3 | ✅ | Timeout I2C INA226 de 1000 ms à 2 ms (`ina226.h:58`). Timeout SPI BMI088 de `HAL_MAX_DELAY` à 2 ms, et suppression des attentes `GetState` sans borne. Les chip ID BMI088 sont vérifiés (lecture fictive après soft reset pour repasser l'accéléro en SPI). En cas d'échec : IMU désactivée, calibration sautée, message ITM. Retours de `INA226_Init`, `BMI088_Read*` et `HAL_TIM_Base_Start_IT` vérifiés. |
| 4 | ⏳ | Demande des mesures sur le robot. |
| 5 | 🟡 partiel | **Resync sur LEN invalide** : un LEN impossible renvoie maintenant `BAD_HEADER` au lieu de `FRAME_SHORT`, donc le parseur avance d'un octet au lieu d'attendre le timeout. Le correctif est dans la fonction partagée `STS3215_ParseReply`, appliqué au firmware et à la lib amont `stm32-sts3215-lib`, avec un test hôte ajouté à `stm32-sts3215-lib/tests/test_protocol.c` : il échoue sans le correctif et passe avec (145 vérifications). **DMA RX servo circulaire : reporté**, voir plus bas. |

Hors Top 5 : les docs mentionnaient un bus « RS485 ». Elles ont été corrigées en « bus série TTL half-duplex » dans `README.md`, dans `Core/Inc/sts3215_hal.h`, et dans le dépôt `chroma` (`actuators.md` FR/EN, `protocol.md` FR).

### Deuxième passe : faisable sans robot

| Constat | État | Changement |
|---|---|---|
| RX bridge sans détection de débordement (Majeur) | ✅ | Compteurs d'octets libres (`rx_written`/`rx_read`) à la place des index : un tour complet du DMA est détecté (`g_bridge_rx_overflow_count`) et le contenu périmé est jeté, jamais rejoué. Un réarmement après erreur UART est resynchronisé. Section critique dans `Bridge_RxEventCallback`, parce que l'IDLE (prio 1) peut préempter le HT/TC (prio 5). Test hôte : `firmware/chromapi_stm32_core/tests/test_bridge_rx.c`, avec HAL bouchonnée, 5 scénarios dont le chien de garde de liaison, propre sous ASan et UBSan. |
| `led_render` annule et perd la demande (Mineur) | ✅ | Le transfert en cours va au bout, puis se relance avec les dernières couleurs (`render_pending`). |
| `g_led_override_until_ms` jamais lu (Mineur) | ✅ | Supprimé. L'override n'avait pas lieu d'être : la LED batterie ne touche que l'index 0 et l'anneau garde ses couleurs dans `rgb_arr`. |
| `g_robot_state` lu sans barrière (Avis) | ✅ | `__COMPILER_BARRIER()` en tête de `Bridge_SendStateSnapshot`. |
| Diagramme d'états divergent (Mineur) | ✅ | Le commentaire de `sts3215_hal.c` décrit maintenant l'armement RX dans `SendFrame`, avant la TX. |
| printf/scanf float liés pour rien | ✅ | Options décochées dans `.cproject` : flash de 72,9 KB à 57,2 KB (−15,7 KB). |

**Reporté, avec la raison :**
- **DMA RX servo circulaire** (Top 5) : c'est un RISQUE déduit du code, pas une panne observée, et il touche le chemin le plus critique (le bus servo). Il faut aussi passer le DMA en circulaire dans le `.ioc` et aligner la priorité NVIC du canal DMA RX sur l'USART2. Sans le robot pour valider la SyncRead, le risque de régression dépasse le gain. À faire avec le robot sur le banc, en regardant `servo_fresh_mask` et `g_servo_err_count`.
- **Servos absents et cycle qui saute** (Top 4) : cela ne se règle bien qu'en connaissant le comportement réel du bus, à mesurer d'abord.
- **Commandes de config non bloquantes et découpage main/bridge** : refactor sans bug observé. L'IWDG couvre désormais le pire blocage (~225 ms).
- **Synchro IMU sur l'INT data-ready** : change le timing de l'IMU, à valider sur le robot.
- **Calibration accéléro « à plat »** : choix documenté dans le cahier de concepts (chapitre 7, « Deux limites à connaître »). Le modifier demande une validation.
- **`NUM_BPP` WS2812B/SK6812, précision du HSI** : il faut vérifier le matériel.
- **Élagage du code mort des libs** (ponytail n° 3 à 5) : ce sont des copies de libs (`stm32-sts3215-lib`, AutoFox INA226, BMI088). Le link en `--gc-sections` les retire déjà du binaire (vérifié avec `nm`), donc il n'y a aucun coût en flash. Les élaguer ferait diverger les copies de leur amont.

### Mesures sur le robot (2026-10-24, firmware de la 2e passe)

| Mesure | Résultat | Conséquence |
|---|---|---|
| `hardware_diagnostic.py` | IMU active (Z = 9,72 m/s²), INA226 OK, 12/12 servos | La vérification du chip ID du BMI088 passe. |
| Chien de garde de liaison (`kill -9` du SDK) | Couple relâché ; à la reconnexion, les servos reprennent le couple dès la première consigne | Aucun `SET_TORQUE` n'est nécessaire côté SDK. |
| IWDG pendant `scan_motors` / `hardware_diagnostic` | Pas de reset | La marge de 500 ms couvre les commandes bloquantes. |
| Return Delay (reg. 0x07) | 0 µs sur les 12 servos | Le budget de poll tient. |
| PING_BRIDGE | 14 195/14 195, RTT p99 = 1,3 ms | — |
| Flux d'état (repos) | 1 001 trames, 0 trou de `seq`, **0 % de servo stale** (12 012 lectures), cycle MCU de 5,000 ms, bus de 3,2 ms en moyenne (max 3,6 ms) | Pas de perte sur la SyncRead. **Le risque « DMA RX servo réarmé » ne se manifeste pas** : le passage en DMA circulaire est rétrogradé (rien à corriger tant que `stale` reste à 0 en marche). |
| Boucle du Pi (100 Hz) | 99,7 Hz, p99 = 10,1 ms, **un tick de 52-54 ms par run** (0,066 %) | À trancher : démarrage du bench ou pic périodique. Le second serait une source de saccades côté Pi. |
| Trot 0,05 m/s, 34 s (`--stand --gait trot`) | 99,9 Hz, gigue σ = 0,7 ms, 199,8 trames d'état/s, 0 % de perte. **Toujours un seul tick long (51 ms) sur 3 433** | Le tick long est un artefact du démarrage du bench (1 par run, quelle que soit la durée), pas un pic périodique. Le cycle de 200 Hz tient **avec** la SyncWrite en marche. Côté firmware et liaison, rien n'explique des saccades. |
| Trot **au sol**, 10 s (impacts de pattes) | 99,7 Hz, 199,5 trames d'état/s, 0 % de perte, un seul tick long (démarrage). Puissance p99 = 22,8 W, max = 27,1 W aux impacts | La chaîne MCU ↔ Pi tient sous impacts. **Aucune saccade ressentie.** |

### Troisième passe : SDK et lib amont (2026-10-25)

| Sujet | Changement |
|---|---|
| Scripts SDK vs chien de garde de liaison | Nouvelle méthode `BridgeClient.sleep()` (`chromapi/src/chromapi/hardware/motherboard_bridge.py`) : elle attend en envoyant un `PING_BRIDGE` sans réponse toutes les 50 ms, et les ACK sont jetés par le `_drain()` de la commande suivante. Test : `test_sleep_feeds_link_watchdog`. Branchée dans `calibrate_servo_centers.py` (le baseline de charge était mesuré sur un servo déjà relâché, ce qui faussait les butées), dans `check_snapshot_decoding.py` (la phase « couple activé » ne lisait que le flux) et dans `servo_step_test.py` (la consigne est renvoyée à chaque snapshot pendant les maintiens, plus `bridge.sleep` dans `probe_goal`). `check_feedback_encoding.py` lit des registres en continu : rien à changer. |
| Écarts firmware → `stm32-sts3215-lib` | Reportés en version générique : `prv_arm_rx` (interruption d'erreur UART coupée, drapeaux nettoyés), resync octet par octet du parseur, réarmement sans `memset`, et `STS3215_RX_BUF_SIZE` calculé depuis `STS3215_REPLY_DATA_MAX` au lieu de 16 figé. La lib garde `REPLY_DATA_MAX = 8`, le firmware 16. Tests hôte : 145/145. Compilation ARM de la lib contre les en-têtes ST : OK. |

### Quatrième passe : servos absents (2026-10-25)

CORDIC décoché dans CubeMX : la régénération a retiré la MSP et les drivers CORDIC, et les blocs `USER CODE` sont intacts (build OK, 57,7 KB de flash).

| Constat | Changement |
|---|---|
| Servo absent au boot exclu à vie ; servo qui décroche = timeout à chaque cycle (Majeur) | Masque de présence (`g_servo_present_mask`) à la place de la liste figée. Un servo sort de la SyncRead après 3 polls manqués d'affilée. Les absents sont re-pingés un par un, une fois par seconde, en fin de cycle, sans bloquer (`SERVO_TXN_RESCAN`) ; la réponse le réintègre. Un poll où **aucun** servo ne répond est traité comme un glitch de bus et ne compte pas de raté. Sans aucun servo présent, l'état (IMU, alimentation) est quand même envoyé au Pi à chaque cycle, alors qu'avant le flux s'arrêtait. |

**Reste de l'audit, non traité par choix** (aucun bug observé, mesures saines) :
- Synchro IMU sur l'interruption data-ready : effet attendu négligeable sur Mahony (Kp = 1, correction lente). À faire si la qualité d'attitude pose problème.
- ~~Découpage `main.c` / `bridge.c`~~ : fait, voir la cinquième passe.
- Commandes de config non bloquantes : le pire blocage (~225 ms) est couvert par l'IWDG (500 ms), et le débordement RX est désormais détecté.
- DMA RX servo circulaire : 0 % de stale mesuré au repos et en trot.
- Calibration accéléro « à plat » : choix documenté dans le cahier de concepts.

**Validation sur le robot (2026-10-25)** :
- **Débranchement/rebranchement d'une patte** (`bridge_bench --skip-loop --wire-duration 20`) : le cycle MCU reste à 5,000 ms (max 6,8 ms), sans trou de `seq`. Les deux manipulations rapprochées ont produit des faux contacts : un servo intermittent (~67 % de cycles en timeout, 5,6 % de stale) n'est jamais retiré, parce qu'il ne rate jamais 3 polls d'affilée. Même dans ce cas, le bus reste sous 5 ms sans SyncWrite. Comme c'est un défaut matériel et non un cas nominal, la règle de retrait n'a pas été durcie.
- **Run propre, sans manipulation** (30 s de flux) : 6 001 trames, 0 % de stale, 0 trou de `seq`, bus à 3,2 ms. Ping : 1 timeout (50 ms) sur 87 652, sans effet possible sur le chien de garde de liaison (100 ms).

**État de l'audit** : tous les constats Critique et Majeur sont corrigés, ou rétrogradés au vu des mesures. Ce qui reste relève de l'avis ou du conditionnel (voir « Reste de l'audit, non traité par choix »).

### Cinquième passe : découpage main / bridge / servo_bus (2026-10-25)

Le bus servo était réparti entre `main.c` (handle, cycle, poll, présence) et `bridge.c` (transactions de config bloquantes, consignes en attente), avec 8 `extern` de `bridge.c` vers les globales de `main.c`. Nouveau découpage, sans changer le comportement :

| Fichier | Rôle | Dépend de |
|---|---|---|
| `servo_bus.c/.h` (nouveau) | Seul propriétaire du handle STS3215 : cycle de 200 Hz, SyncWrite des consignes, SyncRead, présence et re-ping des servos, commandes de config bloquantes, compteurs `g_servo_err_count` | `sts3215_hal`, `sts3215_protocol` |
| `bridge.c` | Protocole Pi seul : RX/TX DMA, décodage, commandes → API `ServoBus_*` / LEDs, snapshot, chien de garde de liaison. **Plus aucun `extern`.** | `servo_bus.h`, types IMU/AHRS |
| `main.c` | Init, boucle, IMU + Mahony, cache INA226, LED batterie, IWDG, routage des callbacks HAL UART | tout le reste |

- Le snapshot reçoit ses entrées explicitement : `Bridge_SendStateSnapshot(t_us, cycle_us, &imu, &ahrs, &power)`. `RobotFeedback_t` est maintenant privé au bridge, et le bus expose son propre `ServoBusState_t`.
- **Effet de bord corrigé** : une commande de config qui démarrait juste après la fin d'une SyncRead écrasait l'état de transaction, et le snapshot de ce cycle n'était pas envoyé. `claim_bus()` termine maintenant la transaction en cours avant de prendre le bus.
- **Nouveau test hôte** `tests/test_servo_bus.c` (bus simulé) : présence, retrait après 3 ratés, re-ping, glitch de bus, consignes avant la SyncRead, `ReleaseAll`, cycle sans servo, et l'effet de bord ci-dessus (le test échoue si on retire la correction). Le test du bridge est adapté à la nouvelle API.
- Hors des blocs `USER CODE`, `main.c` n'a pas changé (seule la suppression de CORDIC par CubeMX). Flash : 58,3 KB.

### Sixième passe : ce qui ne sert pas (2026-10-25)

Vérifié sur la netlist de production (`hardware/production/netlist.ipc`) : **PB3 (SWO) n'est relié à rien** (`unconnected-(U7-PB3-Pad26)`), pas plus que PA0 et PA8. Les interruptions IMU (PA11/PA12) sont câblées.

| Élément | Décision |
|---|---|
| Sortie `printf` par ITM/SWO (`_write`, 5 messages de boot) | Supprimée : sans SWO câblé, elle n'arrivait nulle part. Flash de 58,3 KB à 53,6 KB. |
| SYS → Debug « Trace Asynchronous Sw » | Passer en « Serial Wire » dans CubeMX (SWD conservé pour flasher), ce qui libère PB3. |
| NVIC → TIM4 global interrupt | À décocher : le driver LED ne passe que par l'interruption DMA (canal 3). |
| PA11/PA12 en EXTI sans NVIC | Optionnel : `GPIO_Input`. À remettre en EXTI le jour où l'IMU est synchronisée sur data-ready. |
| SYS → Dead battery signals disabled | **À garder** : désactive les pull-downs UCPD sur PB4 (SW_BotLeft) et PB6 (LED). |
| `CoreDebug TRCENA` + DWT dans `main.c` | **À garder** : c'est l'horloge µs (`micros()`), indépendante du SWO. |
| Audio I2S | Le constat « audio absent du firmware » est sans objet : la carte son (MAX98357A, ICS-43434) est gérée par le Raspberry Pi via ALSA (`hardware_diagnostic.py` la détecte), pas par le STM32. |
