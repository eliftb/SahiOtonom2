#!/usr/bin/env python3
"""Slalom durum makinesi — ROS'suz, saf mantık.

SENARYO (koni sayısı SINIRSIZ):
  Şerit takibi ─▶ engeli gör ─▶ (ilk engelde DUR) ─▶ yan şeride geç
              ▲                                              │
              └──────────── şerit değişti, tekrar ───────────┘

  Her engelde bulunulan şeridin TERSİNE geçilir: sağ→sol→sağ→sol...
  Yalnızca İLK engelde durulur; sonrakiler durmadan geçilir.

NASIL DÖNÜYOR:
  Direksiyonun tek sahibi uart_sender_node3.py'deki PID ve o
  /lane/lateral_deviation'ı SIFIRA sürüyor. Şerit değiştirmek için o
  değere KAYDIRMA ekliyoruz; PID aracı kaydırılmış konuma taşıyor.
  Kaydırmayı bırakınca araç bulunduğu şeridin merkezine oturuyor.

SONRAKİ ENGELLERDE NEDEN DURMUYOR:
  Manevra, karar düğümünün acil duruş eşiğinden (1.5 m) DAHA UZAKTA
  (manevra_tetik_m, varsayılan 3.0 m) başlıyor. Araç yana kaymaya
  başlayınca engel koridordan çıkıyor, /obstacle_detected sönüyor ve
  fren hiç devreye girmiyor. Ayrı bir "freni bastır" mekanizması YOK —
  öyle bir şey emniyet katmanını devre dışı bırakmak olurdu.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class Durum(Enum):
    TAKIP = "serit_takibi"          # engel bekliyor
    DURDU = "ilk_engelde_durdu"     # yalnızca ilk engelde
    GECIS = "serit_degistiriyor"


class Serit(Enum):
    SAG = "sag"
    SOL = "sol"


@dataclass
class Cikti:
    kaydirma: float = 0.0
    durum: Durum = Durum.TAKIP
    serit: Serit = Serit.SAG
    gecilen_engel: int = 0
    aciklama: str = ""
    degisti: bool = False


class SlalomMakinesi:
    """Zamana dayalı, döngüsel slalom manevrası.

    Geçişler SÜRE ile bitiyor (gecis_suresi_s). Şerit tespitinden geri
    besleme kullanılmıyor: geçiş sırasında şerit çizgileri kadrajda yer
    değiştirdiği için sapma değeri güvenilmez oluyor. Bilinen bir parkurda
    süre hem öngörülebilir hem tekrarlanabilir.
    """

    def __init__(
        self,
        manevra_tetik_m: float = 3.0,
        durma_suresi_s: float = 2.0,
        gecis_suresi_s: float = 3.0,
        gecis_sapmasi: float = 0.6,
        durma_hiz_esigi: float = 0.05,
        ilk_engelde_dur: bool = True,
        baslangic_serit: Serit = Serit.SAG,
        # Geçiş bittikten sonra bu süre boyunca yeni engel tetiklenmez.
        # Olmazsa, aynı koni geçiş sonunda hâlâ koridorda görünüyorsa
        # araç anında geri dönmeye kalkar ve zikzak yapar.
        gecis_sonrasi_bekleme_s: float = 1.0,
    ):
        self.tetik = manevra_tetik_m
        self.durma_suresi = durma_suresi_s
        self.gecis_suresi = gecis_suresi_s
        self.gecis_sapmasi = gecis_sapmasi
        self.durma_hiz_esigi = durma_hiz_esigi
        self.ilk_engelde_dur = ilk_engelde_dur
        self.baslangic_serit = baslangic_serit
        self.gecis_sonrasi_bekleme = gecis_sonrasi_bekleme_s
        self.sifirla()

    def sifirla(self):
        self.durum = Durum.TAKIP
        self.serit = self.baslangic_serit
        self.gecilen_engel = 0
        self._t0: Optional[float] = None
        self._son_gecis_bitti: Optional[float] = None

    # ---------------- ana adim ----------------

    def adim(self, simdi: float, engel_var: bool, engel_m: Optional[float],
             hiz: Optional[float]) -> Cikti:
        onceki = self.durum
        yakin = engel_var and engel_m is not None and engel_m <= self.tetik

        if self.durum is Durum.TAKIP:
            # Geçişten hemen sonra aynı koniyi yeniden tetiklemeyi engelle.
            sogumada = (self._son_gecis_bitti is not None
                        and simdi - self._son_gecis_bitti < self.gecis_sonrasi_bekleme)
            if yakin and not sogumada:
                if self.gecilen_engel == 0 and self.ilk_engelde_dur:
                    self._gec(Durum.DURDU, simdi)
                    # Bekleme sayacı BURADA başlamaz; araç GERÇEKTEN durunca
                    # başlar. Engeli gördüğü anda başlatmak, araç hâlâ
                    # yavaşlarken manevrayı engele çok yakın tetiklerdi.
                    self._t0 = None
                else:
                    self._gec(Durum.GECIS, simdi)

        elif self.durum is Durum.DURDU:
            durdu = hiz is None or abs(hiz) <= self.durma_hiz_esigi
            if durdu:
                if self._t0 is None:
                    self._t0 = simdi
                elif simdi - self._t0 >= self.durma_suresi:
                    self._gec(Durum.GECIS, simdi)
            else:
                self._t0 = None

        elif self.durum is Durum.GECIS:
            if simdi - self._t0 >= self.gecis_suresi:
                # Şerit değişti, sayaç arttı, tekrar takibe dön.
                self.serit = Serit.SOL if self.serit is Serit.SAG else Serit.SAG
                self.gecilen_engel += 1
                self._son_gecis_bitti = simdi
                self._gec(Durum.TAKIP, simdi)

        return Cikti(
            kaydirma=self._kaydirma(),
            durum=self.durum,
            serit=self.serit,
            gecilen_engel=self.gecilen_engel,
            aciklama=self._aciklama(simdi, engel_m),
            degisti=(self.durum is not onceki),
        )

    # ---------------- ic ----------------

    def _gec(self, yeni: Durum, simdi: float):
        self.durum = yeni
        self._t0 = simdi

    def _hedef_yon(self) -> Serit:
        """Geçişte gidilecek şerit: bulunulanın tersi."""
        return Serit.SOL if self.serit is Serit.SAG else Serit.SAG

    def _kaydirma(self) -> float:
        """Sapmaya eklenecek değer.

        POZİTİF sapma = "araç solda, SAĞA kır" (uart_sender_node3.py PID
        sözleşmesi). Sola gitmek için NEGATİF kaydırma gerekir.
        """
        if self.durum is not Durum.GECIS:
            return 0.0
        return -self.gecis_sapmasi if self._hedef_yon() is Serit.SOL else +self.gecis_sapmasi

    def _aciklama(self, simdi: float, engel_m) -> str:
        m = f"{engel_m:.2f} m" if engel_m is not None else "-"
        if self.durum is Durum.GECIS:
            kalan = max(0.0, self.gecis_suresi - (simdi - self._t0))
            yon = "SOLA" if self._hedef_yon() is Serit.SOL else "SAĞA"
            return (f"{yon} geçiş | kalan {kalan:.1f} sn | "
                    f"kaydırma {self._kaydirma():+.2f}")
        if self.durum is Durum.DURDU:
            if self._t0 is None:
                return f"engel {m} | duruyor..."
            return f"engel {m} | bekleme {simdi - self._t0:.1f}/{self.durma_suresi:.1f} sn"
        return (f"{self.serit.value} şerit takibi | engel {m} | "
                f"geçilen: {self.gecilen_engel}")
