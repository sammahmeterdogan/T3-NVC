# Git Branch Stratejisi - T3-NVC

## 📊 Branch Yapısı

```
main (production)
  └── development (active development)
      └── feature/* (özellik dalları)
      └── bugfix/* (hata düzeltme dalları)
      └── hotfix/* (acil düzeltmeler)
```

## 🌿 Mevcut Branch'ler

### `main` - Production Branch
- **Durum:** ✅ Stable
- **Son Commit:** `95da39c` - Merge t3sim
- **Amaç:** Production-ready kod
- **Koruma:** Direct push **YOK** - sadece PR ile merge

### `development` - Active Development Branch
- **Durum:** ✅ Active
- **Son Commit:** `95da39c` (main ile sync)
- **Amaç:** Güncel geliştirme çalışmaları
- **İzin:** Direct push izinli (küçük değişiklikler için)

### `t3sim` - Completed Feature Branch
- **Durum:** ✅ Merged to main
- **Amaç:** Turtlesim, Examples ve WebSocket fix'leri
- **Not:** Artık development yerine kullanılmayacak

## 🚀 Workflow

### Yeni Özellik Geliştirme

```bash
# 1. Development'tan feature branch oluştur
git checkout development
git pull origin development
git checkout -b feature/yeni-ozellik-adi

# 2. Geliştirme yap
git add .
git commit -m "feat: Yeni özellik açıklaması"

# 3. Push et
git push -u origin feature/yeni-ozellik-adi

# 4. Development'a merge et
git checkout development
git merge feature/yeni-ozellik-adi
git push origin development

# 5. Test başarılı ise main'e merge (PR ile)
```

### Hata Düzeltme

```bash
# 1. Bugfix branch oluştur
git checkout development
git checkout -b bugfix/hata-aciklamasi

# 2. Fix yap
git add .
git commit -m "fix: Hata açıklaması"

# 3. Development'a merge
git checkout development
git merge bugfix/hata-aciklamasi
git push origin development
```

### Acil Production Fix (Hotfix)

```bash
# 1. Main'den hotfix branch oluştur
git checkout main
git pull origin main
git checkout -b hotfix/kritik-hata

# 2. Fix yap
git add .
git commit -m "hotfix: Kritik hata düzeltmesi"

# 3. ÖNCE main'e merge
git checkout main
git merge hotfix/kritik-hata
git push origin main

# 4. SONRA development'a da merge (sync için)
git checkout development
git merge hotfix/kritik-hata
git push origin development
```

## 📝 Commit Mesaj Kuralları

### Format
```
<type>: <subject>

<body>

<footer>
```

### Type'lar
- `feat`: Yeni özellik
- `fix`: Hata düzeltme
- `docs`: Dokümantasyon
- `style`: Formatting, noktalı virgül eksik vb (kod değişikliği yok)
- `refactor`: Refactoring (ne bug fix ne feature)
- `perf`: Performance iyileştirmesi
- `test`: Test ekleme/düzeltme
- `chore`: Build process, dependency güncellemeleri
- `ci`: CI/CD değişiklikleri

### Örnekler
```bash
# İyi
git commit -m "feat: Add ROS2 Galactic support"
git commit -m "fix: WebSocket reconnection loop"
git commit -m "docs: Update API documentation"

# Kötü
git commit -m "update"
git commit -m "fix bug"
git commit -m "changes"
```

## 🔄 Merge Stratejisi

### Development → Main
- **Yöntem:** Pull Request (PR)
- **Review:** Gerekli (kod review)
- **Test:** Tüm testler geçmeli
- **Zaman:** Sprint sonu veya major feature tamamlandığında

### Feature → Development
- **Yöntem:** Direct merge veya PR (büyük feature'larda)
- **Review:** Opsiyonel
- **Test:** Minimal test yeterli

## 📦 Release Süreci

### Versiyon Numaralandırma: Semantic Versioning
```
MAJOR.MINOR.PATCH
  1  .  2  .  3

MAJOR: Breaking changes
MINOR: Yeni özellikler (backward compatible)
PATCH: Bug fixes
```

### Release Adımları
```bash
# 1. Development'ı main'e merge et
git checkout main
git merge development --no-ff -m "Release v1.2.0"

# 2. Tag oluştur
git tag -a v1.2.0 -m "Release v1.2.0: Yeni özellikler listesi"

# 3. Push et
git push origin main
git push origin v1.2.0

# 4. GitHub'da Release Notes oluştur
```

## 📊 Mevcut Durum (03.01.2026)

### Tamamlanan Özellikler (main branch)
- ✅ **Turtlesim Web Integration** (23848c9)
  - Web-based turtle control
  - VNC visualization
  - Joystick support
  
- ✅ **Examples Page Enhancement** (5e25492)
  - 13 official ROS/ROS2 examples
  - Real documentation links
  - Category filters
  
- ✅ **WebSocket Robustness** (d3ad1b7)
  - Connection retry with backoff
  - State machine
  - Error boundary
  - Subscription queueing

### İstatistikler
- **28 dosya değişti**
- **+2490 satır eklendi**
- **-167 satır silindi**
- **Net: +2323 satır**

## 🎯 Gelecek Planlama

### Development Branch'de Çalışılacaklar
- [ ] Nav2 integration improvements
- [ ] SLAM real-time visualization
- [ ] Map editor UI
- [ ] Performance optimizations
- [ ] Unit test coverage

### Önerilen Feature Branch'ler
```
feature/nav2-waypoint-editor    # Waypoint editing UI
feature/slam-visualization      # Real-time SLAM viz
feature/map-editor             # Interactive map editor
feature/multi-robot            # Multi-robot support
bugfix/teleop-latency          # Reduce teleop latency
```

## 📚 Branch Referansları

### GitHub Links
- **Main:** https://github.com/sammahmeterdogan/T3-NVC/tree/main
- **Development:** https://github.com/sammahmeterdogan/T3-NVC/tree/development
- **All Branches:** https://github.com/sammahmeterdogan/T3-NVC/branches

### Useful Commands
```bash
# Tüm branch'leri göster
git branch -a

# Branch graph
git log --oneline --graph --all

# Remote branch'leri güncelle
git fetch --all --prune

# Development'ı main'den güncelle
git checkout development
git merge main

# Eski branch'leri temizle
git branch -d feature/eski-feature
git push origin --delete feature/eski-feature
```

## ⚠️ Önemli Notlar

1. **Main'e direct push YOK** - Sadece PR ile
2. **Development aktif branch** - Günlük çalışmalar burada
3. **Feature branch'ler kısa ömürlü** - Merge sonrası sil
4. **Commit mesajları anlamlı** - Conventional Commits
5. **Regular sync** - Development'ı main'den düzenli güncelle

---

**Son Güncelleme:** 03.01.2026  
**Branch Yöneticisi:** Development Team  
**Stratejinin Versiyonu:** 1.0.0

