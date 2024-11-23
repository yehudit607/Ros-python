
# מערכת ניהול בתי דין - ארכיטקטורה מבוססת מיקרו-שירותים

## **תיאור המערכת**
מערכת ניהול בתי הדין נועדה לנהל באופן מקיף את מחזור החיים של תיקים משפטיים, החל מהגשת התיק, דרך ניהול דיונים, מסמכים, החלטות ובקשות, ועד הפצת הודעות לגורמים הרלוונטיים. המערכת בנויה על ארכיטקטורת מיקרו-שירותים, תוך הקפדה על חלוקה לוגית, תקשורת יעילה בין רכיבי המערכת, ואבטחה גבוהה.

---

## **חלוקת המערכת למיקרו-שירותים**

### **Core Services** (שירותי ליבה)

#### **Case Service** - ניהול מחזור חיי התיק
- יצירת תיק חדש
- עדכון סטטוס תיק
- ניהול החלטות ובקשות ביניים

#### **User Service** - ניהול משתמשים
- שמירת נתוני משתמשים ותפקידים
- ניהול הרשאות על בסיס OAuth 2.0

#### **Document Service** - ניהול מסמכים
- העלאה, שמירה וגרסאות של מסמכים
- אחסון המסמכים ב-Amazon S3 עם מטא-דאטה ב-PostgreSQL

#### **Hearing Service** - ניהול דיונים
- תזמון דיונים
- ניהול פרוטוקולים וזימון משתתפים

---

### **Supporting Services** (שירותים תומכים)

#### **Notification Service** - שירות התראות
- שליחת מיילים, הודעות SMS והתראות מערכת
- תיעדוף הודעות (High Priority/Low Priority)

#### **Audit Service** - שירות מעקב
- תיעוד פעולות משתמשים ושירותים
- שמירת לוגים ב-Elasticsearch לצורך חיפוש וניתוח

#### **Search Service** - שירות חיפוש
- אינדוקס תיקים, מסמכים ודיונים ב-Elasticsearch
- מענה על חיפושים טקסטואליים וסינונים מתקדמים

---

## **תקשורת בין רכיבי המערכת**

### **API Gateway**
- משמש כשכבת כניסה מאובטחת לכלל השירותים.
- תומך ב-Rate Limiting, אימות באמצעות JWT והפניית קריאות לשירותים המתאימים.

### **Message Queue**
- **RabbitMQ**: נבחר לניהול התראות.
- **Kafka**: נבחר לתיעוד פעולות ולוגים.

### **קריאות סינכרוניות**
- מתבצעות ב-REST (או GraphQL אם נדרש חיפוש גמיש יותר).
- בין שירותי ליבה, לדוגמה: Case Service פונה ל-User Service לאימות נתונים.

### **קריאות אסינכרוניות**
- תור ההודעות (RabbitMQ/Kafka) משמש לתקשורת שאינה דורשת תגובה מיידית.
- מבטיח עמידות בתקלות, עם מנגנון Retry מובנה.

---

## **שיקולים טכנולוגיים**
- **אבטחה**: שימוש ב-HTTPS להצפנת תקשורת, אימות משתמשים ב-OAuth 2.0, ו-API Gateway להגבלת גישה וניהול הרשאות.
- **מסדי נתונים**:  
  - **PostgreSQL**: לניהול נתונים רלציוניים (תיקים, משתמשים).
  - **Elasticsearch**: לחיפושים מהירים.
  - **Redis**: לניהול תורים זמניים (Cache) ולשיפור זמני תגובה.
- **סקלאביליות**: כל שירות ניתן להרחבה עצמאית בהתאם לעומסים.
- **ניטור ובקרה**: Prometheus ו-Grafana לניטור ביצועים, ו-Zipkin לטרייסינג בין שירותים.

---

## **תרשים ארכיטקטורה (Mermaid)**

```mermaid
graph TD
    UI[Client (Web/Frontend)]
    API[API Gateway]

    UI --> API
    API --> CaseService[Case Service]
    API --> UserService[User Service]
    API --> DocumentService[Document Service]
    API --> HearingService[Hearing Service]
    API --> NotificationService[Notification Service]
    API --> AuditService[Audit Service]
    API --> SearchService[Search Service]

    subgraph Async Communication
        CaseService -->|RabbitMQ| NotificationService
        CaseService -->|Kafka| AuditService
        DocumentService -->|Kafka| AuditService
        HearingService -->|RabbitMQ| NotificationService
        HearingService -->|Kafka| AuditService
    end

    subgraph Sync Communication
        CaseService -->|REST| UserService
        CaseService -->|REST| DocumentService
        HearingService -->|REST| CaseService
    end

    SearchService -->|Elasticsearch| AuditService
```

---

## **יתרונות הארכיטקטורה**
1. **הפרדה ברורה**: חלוקה ברורה לתחומי אחריות מונעת תלותיות בין שירותים.
2. **גמישות טכנולוגית**: כל שירות משתמש בטכנולוגיות המתאימות לצרכיו.
3. **יעילות בתקשורת**: שילוב של תורים לקריאות אסינכרוניות ו-REST לקריאות סינכרוניות.
4. **יכולת הרחבה**: כל רכיב ניתן להרחבה עצמאית.
5. **אבטחה חזקה**: שילוב של הצפנה, אימות והרשאות ברמת השירות.

ארכיטקטורה זו מאפשרת תפעול יעיל ומאובטח של מערכת מורכבת עם פוטנציאל לגידול עתידי.
