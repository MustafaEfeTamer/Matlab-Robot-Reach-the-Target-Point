
% Harita boyutlarını giriyorum
haritaBoyutu = [20, 20];
map = zeros(haritaBoyutu); % 0: boş alan, 1: engel

% Haritaya rastgele konumlu engeller ekliyorum
engelSayisi = 40;   % Engel sayısı
for i = 1:engelSayisi
    x = randi(haritaBoyutu(1));  % x ini rastgele alıyorum
    y = randi(haritaBoyutu(2));  % y sini rastgele alıyorum
    map(x, y) = 1; % Engel yerleştir
end

% 10 robot için başlangıç ve hedef noktaları belirliyorum
robotSayisi = 10;
baslangicPozisyonu = zeros(robotSayisi, 2); % 10x2 lik bir matris oluşturur ve her robotun başlangıç x y koordinatları tutulur
hedefPozisyon = zeros(robotSayisi, 2); % % 10x2 lik bir matris oluşturur ve her robotun hedef x y koordinatları tutulur

for i = 1 : robotSayisi
    % Rastgele başlangıç noktası belirliyorum ve engelin üzerinde mi diye
    % kontrol ediyorum 
    while true
        basNokta = [randi(haritaBoyutu(1)), randi(haritaBoyutu(2))];
        if map(basNokta(1), basNokta(2)) == 0
            baslangicPozisyonu(i, :) = basNokta;
            break;
        end
    end
    
    % Rastgele hedef noktası belirliyorum ve engelin üzerinde mi diye
    % kontol ediyorum
    while true
        hedefNokta = [randi(haritaBoyutu(1)), randi(haritaBoyutu(2))];
        if map(hedefNokta(1), hedefNokta(2)) == 0
            hedefPozisyon(i, :) = hedefNokta;
            break;
        end
    end
end

% Haritayı görselleştiriyorum
figure('Position', [100, 100, 1000, 1000]); % pencere oluşturuyorum
imagesc(map); colormap(gray); axis equal;
hold on;

% her robotun başlangıç ve hedef noktalarını çiziyorum
for j = 1:robotSayisi
    plot(baslangicPozisyonu(j, 2), baslangicPozisyonu(j, 1), '*g', 'MarkerSize', 10, 'LineWidth', 2); % Başlangıç
    plot(hedefPozisyon(j, 2), hedefPozisyon(j, 1), 'xr', 'MarkerSize', 10, 'LineWidth', 2); % Hedef
end

% Izgara görünümü ve başlık ekliyorum
grid on;
xticks(1 : haritaBoyutu(2));
yticks(1 : haritaBoyutu(1));
title('10 adet robot için rota planması');
xlabel('X Koordinatı');
ylabel('Y Koordinatı');

yollar = cell(1, robotSayisi); 
renkler = hsv(robotSayisi);  % yolların renklerin veriyorum

L = 0.5;  % Tekerlekler arasındaki mesafeyi giriyorum metre cinsinden
maxhiz = 0.05;  % maksimum hız degerim
maxIvme = 0.05;  % maksimum ivme degerim
yolSuresi = 50;  % robotların hareket süresini giriyorum

% Her robot için yol planlama
for j = 1 : robotSayisi
    yollar{j} = aStar(map, baslangicPozisyonu(j, :), hedefPozisyon(j, :));
end

% Hız profilleri ve tekerlek hızlarını hesaplıyorum
v = cell(1, robotSayisi);
w = cell(1, robotSayisi);
sol_hiz = cell(1, robotSayisi);  % Her robot için sol tekerlek hızlarını saklayacak
sag_hiz = cell(1, robotSayisi);  % Her robot için sağ tekerlek hızlarını saklayacak

for j = 1 : robotSayisi
    if ~isempty(yollar{j})
        % Hız profillerini oluşturuyorum
        [v{j}, w{j}] = hizProfiliniOlustur(baslangicPozisyonu(j, :), hedefPozisyon(j, :), maxhiz, maxIvme, yolSuresi);
        
        % Tekerlek hızlarını hesaplıyorum
        [sol_hiz{j}, sag_hiz{j}] = tekerlekHiziniHesapla(v{j}, w{j}, L);
    else
        disp(['Robot ', num2str(j), ' için bir yol bulunamadı.']);
    end
end

% Robotları aynı anda hareket ettirmek için animasyon simülasyonu
maxUzunluk = max(cellfun(@length, yollar)); % En uzun yolun uzunluğunu buluyorum
for n = 1:maxUzunluk
    for j = 1 : robotSayisi
        % Robotu güncelle
        if ~isempty(yollar{j}) && n <= length(yollar{j})
            % Robotun yolunu çiz
            plot(yollar{j}(n, 2), yollar{j}(n, 1), 'o', 'Color', renkler(j, :), 'MarkerSize', 5, 'LineWidth', 2);
        end
    end
    pause(0.5);
end

title('10 Robot için Yol Planlama');

% Her robotun hız profillerini tek bir grafikte gösteriyorum
figure('Position', [100, 100, 1200, 800]); 
hold on;

% Her robotun hız profilini çiz
for j = 1 : robotSayisi
    if ~isempty(v{j}) && ~isempty(w{j})
        plot(linspace(0, yolSuresi, 100), v{j}, 'DisplayName', ['Robot ', num2str(j), ' - Çizgisel Hız'], 'LineWidth', 2); % Doğrusal hız
        plot(linspace(0, yolSuresi, 100), w{j}, 'DisplayName', ['Robot ', num2str(j), ' - Açısal Hız'], 'LineWidth', 2); % Açısal hız
    end
end

% Grafik özelliklerini ayarlıyorum
title('Robotların Hız Profilleri');
xlabel('Zaman (saniye)');
ylabel('Hız (m/s veya rad/s)');
legend show;
grid on;
hold off;

% Astar fonksiyonu ile rotayı belirliyorum
function path = aStar(map, basNokta, hedefNokta)
    [rows, cols] = size(map);
    moves = [0 1; 1 0; 0 -1; -1 0]; % 4 komşu
    openList = struct('pos', basNokta, 'g', 0, 'h', yolMaliyetiHesapla(basNokta, hedefNokta), 'parent', []); 
    closedList = [];
    
    while ~isempty(openList)
        [~, idx] = min(arrayfun(@(x) x.g + x.h, openList));
        suanlik = openList(idx);
        if isequal(suanlik.pos, hedefNokta)
            path = yenidenYolYapimi(closedList, suanlik);
            return;
        end
        
        closedList = [closedList, suanlik];
        openList(idx) = [];
        
        for j = 1:size(moves, 1)
            komsuNokta = suanlik.pos + moves(j, :);
            if komsuNokta(1) < 1 || komsuNokta(1) > rows || ...
               komsuNokta(2) < 1 || komsuNokta(2) > cols || ...
               map(komsuNokta(1), komsuNokta(2)) == 1 || ...
               any(arrayfun(@(x) isequal(x.pos, komsuNokta), closedList))
                continue;
            end
            
            maliyet = suanlik.g + 1;
            acikMi = false;
            for j = 1:length(openList)
                if isequal(openList(j).pos, komsuNokta)
                    acikMi = true;
                    if maliyet < openList(j).g
                        openList(j).g = maliyet;
                        openList(j).parent = suanlik;
                    end
                    break;
                end
            end
            
            if ~acikMi
                openList = [openList, struct('pos', komsuNokta, 'g', maliyet, ...
                                             'h', yolMaliyetiHesapla(komsuNokta, hedefNokta), ...
                                             'parent', suanlik)];
            end
        end
    end
    
    path = [];
end

function h = yolMaliyetiHesapla(pozisyon, hedef)
    h = abs(pozisyon(1) - hedef(1)) + abs(pozisyon(2) - hedef(2)); 
end

function yol = yenidenYolYapimi(~, suanlik)
    yol = suanlik.pos;
    while ~isempty(suanlik.parent)
        suanlik = suanlik.parent;
        yol = [suanlik.pos; yol];
    end
end

function [v, w] = hizProfiliniOlustur(baslangicPozisyonu, hedefPozisyonu, maxhiz, maxIvme, yolSuresi)
    % Başlangıç ve hedef noktası arasındaki mesafeyi basitçe hesaplıyorum
    mesafe = norm(hedefPozisyonu - baslangicPozisyonu);
    
    % Hız profili oluşturmak için adım sayısını ve adım süresini alıyorum
    adimSayisi = 100;
    adimSuresi = yolSuresi / adimSayisi;  
    
    % Hız profili yani çizgisel ve açısal hız oluşturuyorum
    v = linspace(0, maxhiz, adimSayisi);  % çizgisel hız degerini hesaplıyorum
    w = linspace(0, maxhiz / mesafe, adimSayisi);  % Açısal hız degerini hesapliyorum 
end


function [sol_hiz, sag_hiz] = tekerlekHiziniHesapla(v, w, L)
    % Sol ve sağ tekerlek hızlarını hesaplıyorum
    sol_hiz = v - ((w * L) / 2);
    sag_hiz = v + ((w * L) / 2);
end


% Çakışan robotları kontrol edip ekranda gösteriyorum
function cakismalariKontrolEt(yollar, robotSayisi)
    cakisiyor = [];
    for i = 1:robotSayisi-1
        for j = i+1:robotSayisi
            % İki robotun yollarının çakışıp çakışmadığını kontrol et
            cakisma = intersect(yollar{i}, yollar{j}, 'rows');
            if ~isempty(cakisma)
                cakisiyor = [cakisiyor; i, j];
            end
        end
    end
    
    % Çakışan robotları yazdır
    if ~isempty(cakisiyor)
        figure('Position', [100, 100, 600, 400]);
        hold on;
        for k = 1:size(cakisiyor, 1)
            text(0.5, 0.9-k*0.1, ['Robot ', num2str(cakisiyor(k, 1)), ' ile Robot ', num2str(cakisiyor(k, 2))], ...
                 'FontSize', 14, 'HorizontalAlignment', 'center');
        end
        axis off;
        title('Yolları Çakışan Robotlar');
    else
        disp('Çakışan hiç robot yok.');
    end
end

% Çakışan robotları kontrol et
cakismalariKontrolEt(yollar, robotSayisi);

